use crate::*;

pub struct LogicAnalyzer {
    serial: SerialPort<'static, UsbBus>,
    usb_dev: UsbDevice<'static, UsbBus>,
    sampler: Sampler,
    trigger: Trigger,
    prescaler: u16,
    flags: u8,
    samples: usize,
    needle: usize,
    scratch: [u8; 64],
}

impl LogicAnalyzer {
    pub fn new(
        usb_dev: UsbDevice<'static, UsbBus>,
        serial: SerialPort<'static, UsbBus>,
        pio: PIO<pac::PIO0>,
        sm: UninitStateMachine<(pac::PIO0, SM0)>,
        dma: dma::Channels,
    ) -> Self {
        let sampler = Sampler::new(pio, sm, dma);
        Self {
            sampler,
            serial,
            usb_dev,
            flags: 0,
            samples: 0,
            prescaler: 0,
            needle: 0,
            scratch: [0; 64],
            trigger: Default::default(),
        }
    }

    pub fn acquisition_done(&mut self) {
        self.sampler
            .drain(&mut self.serial, self.samples, self.flags);
    }

    pub fn poll_serial(&mut self) {
        if self.usb_dev.poll(&mut [&mut self.serial]) {
            match self.parse_command() {
                Some(SumpCommand::Reset) => {
                    self.needle = 0;
                    self.samples = 0;
                }
                Some(SumpCommand::Arm) => self.sampler.start(self.prescaler, self.trigger),
                Some(SumpCommand::SetFlags(flags)) => self.flags = flags,
                Some(SumpCommand::SetPrescaler(prescaler)) => {
                    self.prescaler = prescaler;
                }
                Some(SumpCommand::SetReadCount(amount)) => {
                    self.samples = amount;
                }
                Some(SumpCommand::SetTriggerMask(stage, mask)) if stage < 4 => {
                    self.trigger.set_mask(stage as _, mask);
                }
                Some(SumpCommand::SetTriggerValues(stage, pattern)) if stage < 4 => {
                    self.trigger.set_pattern(stage as _, pattern);
                }
                Some(SumpCommand::SetTriggerDelay(stage, delay)) if stage < 4 => {
                    self.trigger.set_delay(stage as _, delay);
                }
                Some(SumpCommand::GetId) => {
                    self.serial.write(b"1ALS").ok();
                }
                Some(SumpCommand::GetMeta) => {
                    self.serial.write(&[0x01]).ok();
                    self.serial.write(b"uLA: Micro Logic Analyzer").ok();
                    self.serial.write(&[0x00, 0x20]).ok();
                    self.serial.write(&PROBES.to_be_bytes()).ok();
                    self.serial.write(&[0x21]).ok();
                    self.serial.write(&(SAMPLE_MEMORY).to_be_bytes()).ok();
                    self.serial.write(&[0x23]).ok();
                    self.serial.write(&SAMPLE_RATE.to_be_bytes()).ok();
                    self.serial.write(&[0x24, 0x00, 0x00, 0x00, 0x02]).ok();
                }

                _ => {}
            }
        }
    }

    fn parse_command(&mut self) -> Option<SumpCommand> {
        match self.serial.read(&mut self.scratch[self.needle..]) {
            Ok(n) if n > 0 => {
                self.needle += n;
                match self.scratch[0] {
                    0x00 => {
                        self.drain_rx(1);
                        Some(SumpCommand::Reset)
                    }
                    0x01 => {
                        self.drain_rx(1);
                        Some(SumpCommand::Arm)
                    }
                    0x02 => {
                        self.drain_rx(1);
                        Some(SumpCommand::GetId)
                    }
                    0x04 => {
                        self.drain_rx(1);
                        Some(SumpCommand::GetMeta)
                    }
                    cmd if self.needle > 4 => match cmd {
                        0x80 => {
                            let prescaler =
                                u32::from_le_bytes(self.scratch[1..5].try_into().unwrap());
                            self.drain_rx(5);
                            Some(SumpCommand::SetPrescaler(prescaler as _))
                        }
                        0x81 => {
                            let samples =
                                u16::from_le_bytes(self.scratch[1..3].try_into().unwrap());
                            self.drain_rx(5);
                            Some(SumpCommand::SetReadCount(samples as _))
                        }
                        0x82 => {
                            let flags = self.scratch[1];
                            self.drain_rx(5);
                            Some(SumpCommand::SetFlags(flags))
                        }
                        0xc0 | 0xc4 | 0xc8 | 0xcc => {
                            let stage = (self.scratch[0] - 0xc0) / 4;
                            let mask = u32::from_le_bytes(self.scratch[1..5].try_into().unwrap());
                            self.drain_rx(5);
                            Some(SumpCommand::SetTriggerMask(stage, mask))
                        }
                        0xc1 | 0xc5 | 0xc9 | 0xcd => {
                            let stage = (self.scratch[0] - 0xc1) / 4;
                            let val = u32::from_le_bytes(self.scratch[1..5].try_into().unwrap());
                            self.drain_rx(5);
                            Some(SumpCommand::SetTriggerValues(stage, val))
                        }
                        0xc2 | 0xc6 | 0xca | 0xce => {
                            let stage = (self.scratch[0] - 0xc2) / 4;
                            let delay = u16::from_le_bytes(self.scratch[1..3].try_into().unwrap());
                            self.drain_rx(5);
                            Some(SumpCommand::SetTriggerDelay(stage, delay as _))
                        }
                        _ => {
                            self.drain_rx(1);
                            None
                        }
                    },
                    _ => None,
                }
            }
            _ => None,
        }
    }

    fn drain_rx(&mut self, n: usize) {
        self.needle -= n;
        self.scratch.copy_within(n.., 0);
    }
}

enum SumpCommand {
    Reset,
    Arm,
    GetId,
    GetMeta,
    SetPrescaler(u16),
    SetReadCount(usize),
    SetFlags(u8),
    SetTriggerMask(u8, u32),
    SetTriggerValues(u8, u32),
    SetTriggerDelay(u8, u32),
}
