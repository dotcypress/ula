use crate::*;

/// Enumeration of Sump commands used to control the Logic Analyzer.
#[derive(Debug)]
enum SumpCommand {
    /// Reset the analyzer.
    Reset,
    /// Arm the analyzer to start data acquisition.
    Arm,
    /// Retrieve the device ID.
    GetId,
    /// Get metadata information about the analyzer.
    GetMeta,
    /// Set the sampling divisor.
    SetDivisor(u16),
    /// Set the number of samples to read.
    SetReadCount(usize),
    /// Set specific configuration flags.
    SetFlags(u8),
    /// Set the trigger mask for a specific stage.
    SetTriggerMask(u8, u32),
    /// Set the trigger values for a specific stage.
    SetTriggerValues(u8, u32),
    /// Set the trigger delay for a specific stage.
    SetTriggerDelay(u8, u32),
}

/// Type alias for the status LED pin configuration.
pub type Led = Pin<bank0::Gpio25, FunctionSio<SioOutput>, PullDown>;

/// Struct representing the Logic Analyzer device.
pub struct LogicAnalyzer {
    /// Serial communication interface for USB.
    serial: SerialPort<'static, UsbBus>,
    /// USB device object.
    usb_dev: UsbDevice<'static, UsbBus>,
    /// LED to indicate the status of the analyzer.
    status_led: Led,
    /// Sampler responsible for data acquisition.
    sampler: Sampler,
    /// Trigger settings and configurations.
    trigger: Trigger,
    /// Index used for parsing incoming commands.
    needle: usize,
    /// Buffer for storing incoming serial data.
    scratch: [u8; 64],
}

impl LogicAnalyzer {
    /// Creates a new instance of the Logic Analyzer.
    ///
    /// # Arguments
    ///
    /// * `usb_dev` - USB device instance.
    /// * `serial` - Serial port for USB communication.
    /// * `pio` - PIO instance for handling programmable I/O.
    /// * `sm` - Uninitialized state machine for PIO.
    /// * `dma` - DMA channels for data transfer.
    /// * `status_led` - LED pin for status indication.
    ///
    /// # Returns
    ///
    /// A new `LogicAnalyzer` instance.
    pub fn new(
        usb_dev: UsbDevice<'static, UsbBus>,
        serial: SerialPort<'static, UsbBus>,
        pio: PIO<pac::PIO0>,
        sm: UninitStateMachine<(pac::PIO0, SM0)>,
        dma: dma::Channels,
        status_led: Led,
    ) -> Self {
        let sampler = Sampler::new(pio, sm, dma);
        Self {
            sampler,
            serial,
            usb_dev,
            status_led,
            needle: 0,
            scratch: [0; 64],
            trigger: Default::default(),
        }
    }

    /// Called when data acquisition is complete.
    ///
    /// Drains the sampler's data into the serial port and turns off the status LED.
    pub fn acquisition_done(&mut self) {
        self.sampler.drain(&mut self.serial);
        self.status_led.set_low().unwrap();
    }

    /// Polls the serial interface for incoming commands and processes them.
    pub fn poll_serial(&mut self) {
        if self.usb_dev.poll(&mut [&mut self.serial]) {
            // If a new command is received, parse and execute it.
            if let Some(cmd) = self.parse_command() {
                match cmd {
                    SumpCommand::Reset => {
                        // Reset the needle index.
                        self.needle = 0;
                    }
                    SumpCommand::Arm => {
                        // Activate the status LED and start the sampler with the current trigger.
                        self.status_led.set_high().unwrap();
                        self.sampler.start(self.trigger);
                    }
                    SumpCommand::SetFlags(flags) => {
                        // Set configuration flags in the sampler.
                        self.sampler.set_flags(flags);
                    }
                    SumpCommand::SetDivisor(divisor) => {
                        // Set the sampling divisor in the sampler.
                        self.sampler.set_divisor(divisor);
                    }
                    SumpCommand::SetReadCount(samples) => {
                        // Set the number of samples to read in the sampler.
                        self.sampler.set_sample_memory(samples);
                    }
                    SumpCommand::SetTriggerMask(stage, mask) if stage < 4 => {
                        // Set the trigger mask for a specific stage.
                        self.trigger.set_mask(stage as _, mask);
                    }
                    SumpCommand::SetTriggerValues(stage, pattern) if stage < 4 => {
                        // Set the trigger pattern for a specific stage.
                        self.trigger.set_pattern(stage as _, pattern);
                    }
                    SumpCommand::SetTriggerDelay(stage, delay) if stage < 4 => {
                        // Set the trigger delay for a specific stage.
                        self.trigger.set_delay(stage as _, delay);
                    }
                    SumpCommand::GetId => {
                        // Send the device ID over the serial port.
                        self.serial.write(b"1ALS").ok();
                    }
                    SumpCommand::GetMeta => {
                        // Send metadata information over the serial port.
                        self.serial.write(&[0x01]).ok();
                        self.serial.write(b"uLA: Micro Logic Analyzer").ok();
                        self.serial.write(&[0x00, 0x20]).ok();
                        self.serial.write(&PROBES.to_be_bytes()).ok();
                        self.serial.write(&[0x21]).ok();
                        self.serial.write(&(SAMPLE_MEMORY).to_be_bytes()).ok();
                        self.serial.write(&[0x23]).ok();
                        self.serial.write(&SAMPLE_RATE.to_be_bytes()).ok();
                        self.serial
                            .write(&[0x24, 0x00, 0x00, 0x00, 0x02, 0x00])
                            .ok();
                    }
                    _ => {
                        // Ignore unrecognized commands.
                    }
                }
            }
        }
    }

    /// Parses incoming serial data to identify and construct Sump commands.
    ///
    /// # Returns
    ///
    /// An `Option<SumpCommand>` if a complete command is parsed.
    fn parse_command(&mut self) -> Option<SumpCommand> {
        match self.serial.read(&mut self.scratch[self.needle..]) {
            Ok(n) if n > 0 => {
                // Update the needle index based on the number of bytes read.
                self.needle += n;
                // Identify the command based on the first byte.
                match self.scratch[0] {
                    0x00 => {
                        // Reset command.
                        self.drain_rx(1);
                        Some(SumpCommand::Reset)
                    }
                    0x01 => {
                        // Arm command.
                        self.drain_rx(1);
                        Some(SumpCommand::Arm)
                    }
                    0x02 => {
                        // GetId command.
                        self.drain_rx(1);
                        Some(SumpCommand::GetId)
                    }
                    0x04 => {
                        // GetMeta command.
                        self.drain_rx(1);
                        Some(SumpCommand::GetMeta)
                    }
                    cmd if self.needle > 4 => {
                        // Handle more complex commands that require additional bytes.
                        match cmd {
                            0x80 => {
                                // SetDivisor command with a 4-byte prescaler.
                                let prescaler =
                                    u32::from_le_bytes(self.scratch[1..5].try_into().unwrap());
                                self.drain_rx(5);
                                Some(SumpCommand::SetDivisor(prescaler as _))
                            }
                            0x81 => {
                                // SetReadCount command with a 2-byte sample count.
                                let samples =
                                    u16::from_le_bytes(self.scratch[1..3].try_into().unwrap());
                                self.drain_rx(5);
                                Some(SumpCommand::SetReadCount(samples as _))
                            }
                            0x82 => {
                                // SetFlags command with a single byte of flags.
                                let flags = self.scratch[1];
                                self.drain_rx(5);
                                Some(SumpCommand::SetFlags(flags))
                            }
                            0xc0 | 0xc4 | 0xc8 | 0xcc => {
                                // SetTriggerMask command for different stages.
                                let stage = (self.scratch[0] - 0xc0) / 4;
                                let mask =
                                    u32::from_le_bytes(self.scratch[1..5].try_into().unwrap());
                                self.drain_rx(5);
                                Some(SumpCommand::SetTriggerMask(stage, mask))
                            }
                            0xc1 | 0xc5 | 0xc9 | 0xcd => {
                                // SetTriggerValues command for different stages.
                                let stage = (self.scratch[0] - 0xc1) / 4;
                                let val =
                                    u32::from_le_bytes(self.scratch[1..5].try_into().unwrap());
                                self.drain_rx(5);
                                Some(SumpCommand::SetTriggerValues(stage, val))
                            }
                            0xc2 | 0xc6 | 0xca | 0xce => {
                                // SetTriggerDelay command for different stages.
                                let stage = (self.scratch[0] - 0xc2) / 4;
                                let delay =
                                    u16::from_le_bytes(self.scratch[1..3].try_into().unwrap());
                                self.drain_rx(5);
                                Some(SumpCommand::SetTriggerDelay(stage, delay as _))
                            }
                            _ => {
                                // Unknown command, drain one byte and ignore.
                                self.drain_rx(1);
                                None
                            }
                        }
                    }
                    _ => {
                        // Not enough data to parse a command.
                        None
                    }
                }
            }
            _ => {
                // No data read or an error occurred.
                None
            }
        }
    }

    /// Drains `n` bytes from the receive buffer by shifting remaining bytes.
    ///
    /// # Arguments
    ///
    /// * `n` - Number of bytes to drain.
    fn drain_rx(&mut self, n: usize) {
        self.needle -= n;
        self.scratch.copy_within(n.., 0);
    }
}
