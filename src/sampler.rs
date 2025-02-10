use crate::*;

/// Type alias for the ingest tuple containing the state machine and transmitter.
type Ingest = (
    StateMachine<(pac::PIO0, SM0), Running>,
    Tx<(pac::PIO0, hal::pio::SM0)>,
);

/// Enumeration representing the current state of the sink.
enum Sink {
    /// Data transfer is in progress with a single buffer.
    InProgress(
        single_buffer::Transfer<
            Channel<CH11>,
            Rx<(pac::PIO0, SM0)>,
            &'static mut [u32; SAMPLE_MEMORY / 4],
        >,
    ),
    /// Sink is in standby mode, ready to accept new transfers.
    StandBy(
        (
            Channel<CH11>,
            Rx<(pac::PIO0, SM0)>,
            &'static mut [u32; SAMPLE_MEMORY / 4],
        ),
    ),
}

/// Struct representing the Sampler responsible for data acquisition.
pub struct Sampler {
    /// PIO instance used for programmable I/O.
    pio: PIO<pac::PIO0>,
    /// Current sink state.
    sink: Option<Sink>,
    /// Ingest tuple containing the state machine and transmitter.
    ingest: Option<Ingest>,
    /// Divisor used for sampling rate control.
    divisor: u16,
    /// Number of samples to read.
    samples: usize,
    /// Grouping flags for channels.
    ch_groups: [bool; 2],
}

impl Sampler {
    /// Creates a new instance of the Sampler.
    ///
    /// # Arguments
    ///
    /// * `pio` - PIO instance for programmable I/O.
    /// * `sm` - Uninitialized state machine for PIO.
    /// * `dma` - DMA channels for data transfer.
    ///
    /// # Returns
    ///
    /// A new `Sampler` instance.
    pub fn new(
        pio: PIO<pac::PIO0>,
        sm: UninitStateMachine<(pac::PIO0, SM0)>,
        dma: dma::Channels,
    ) -> Self {
        let mut dma_ch = dma.ch11;
        dma_ch.enable_irq0();

        let mut pio = pio;
        let mut asm = TriggerAssembler::new();
        asm.push(true, true);
        let program = pio.install(&asm.assemble_program()).unwrap();
        let (sm, rx, tx) = PIOBuilder::from_installed_program(program)
            .out_shift_direction(ShiftDirection::Left)
            .build(sm);
        let sm = sm.start();

        // Allocate memory for sample storage using a singleton.
        let samples = singleton!(: [u32; SAMPLE_MEMORY / 4] = [0x00; SAMPLE_MEMORY / 4]).unwrap();
        let sink = Sink::StandBy((dma_ch, rx, samples));

        Self {
            pio,
            divisor: 0,
            samples: 0,
            ch_groups: [false; 2],
            ingest: Some((sm, tx)),
            sink: Some(sink),
        }
    }

    /// Sets the configuration flags for channel groups.
    ///
    /// # Arguments
    ///
    /// * `flags` - Bitmask representing the configuration flags.
    pub fn set_flags(&mut self, flags: u8) {
        self.ch_groups[0] = flags >> 2 & 1 == 0;
        self.ch_groups[1] = flags >> 3 & 1 == 0;
    }

    /// Sets the sampling divisor to control the sampling rate.
    ///
    /// # Arguments
    ///
    /// * `divisor` - The divisor value to set.
    pub fn set_divisor(&mut self, divisor: u16) {
        self.divisor = divisor;
    }

    /// Sets the number of samples to store in memory.
    ///
    /// # Arguments
    ///
    /// * `samples` - The number of samples to read.
    pub fn set_sample_memory(&mut self, samples: usize) {
        self.samples = samples;
    }

    /// Starts the data acquisition process with the specified trigger configuration.
    ///
    /// # Arguments
    ///
    /// * `trigger` - The trigger configuration to use.
    pub fn start(&mut self, trigger: Trigger) {
        // Retrieve the current DMA channel and PIO resources.
        let (ch, rx, sample_mem) = match self.sink.take() {
            Some(Sink::StandBy(dma)) => dma,
            Some(Sink::InProgress(tx)) => tx.abort(),
            _ => unreachable!(),
        };

        // Initialize the state machine and install the new PIO program based on the trigger.
        match self.ingest.take() {
            Some((sm, tx)) => {
                let (sm, old) = sm.uninit(rx, tx);
                self.pio.uninstall(old);
                let program = trigger.compile();
                let program = self.pio.install(&program).unwrap();
                let (sm, rx, tx) = PIOBuilder::from_installed_program(program)
                    .out_shift_direction(ShiftDirection::Left)
                    .clock_divisor_fixed_point(self.divisor + 1, 0)
                    .autopush(true)
                    .in_pin_base(PIN_BASE as _)
                    .build(sm);
                let mut transfer = single_buffer::Config::new(ch, rx, sample_mem);
                transfer.pace(Pace::PreferSource);
                let transfer = transfer.start();
                self.sink = Some(Sink::InProgress(transfer));
                self.ingest = Some((sm.start(), tx));
            }
            _ => unreachable!(),
        }
    }

    /// Drains the acquired data and sends it over the serial port.
    ///
    /// # Arguments
    ///
    /// * `serial` - Mutable reference to the serial port for data transmission.
    pub fn drain(&mut self, serial: &mut SerialPort<'_, UsbBus>) {
        if let Some(sink) = self.sink.take() {
            match sink {
                Sink::StandBy((mut ch, rx, sample_mem)) => {
                    ch.check_irq0();
                    self.sink = Some(Sink::StandBy((ch, rx, sample_mem)));
                }
                Sink::InProgress(mut tx) => {
                    tx.check_irq0();
                    let (ch, rx, sample_mem) = tx.abort();
                    // Iterate over the sample memory and send data based on channel groups.
                    for chunk in sample_mem.chunks(2).take(self.samples + 1).rev() {
                        let s02 = chunk[1].to_le_bytes();
                        let s13 = chunk[0].to_le_bytes();
                        match self.ch_groups {
                            [true, false] => {
                                // Send specific bits for channel group 0.
                                serial.write(&[s02[0], s02[2], s13[0], s13[2]]).ok();
                            }
                            [false, true] => {
                                // Send specific bits for channel group 1.
                                serial.write(&[s02[1], s02[3], s13[1], s13[3]]).ok();
                            }
                            [true, true] => {
                                // Send all bits if both channel groups are active.
                                serial.write(&s02).ok();
                                serial.write(&s13).ok();
                            }
                            _ => {
                                // Do not send data if no channel groups are active.
                            }
                        }
                    }
                    // Return the DMA channel and sample memory to standby.
                    self.sink = Some(Sink::StandBy((ch, rx, sample_mem)));
                }
            }
        }
    }
}
