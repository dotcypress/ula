use crate::*;

type Ingest = (
    StateMachine<(pac::PIO0, SM0), Running>,
    Tx<(pac::PIO0, hal::pio::SM0)>,
);

enum Sink {
    InProgress(
        single_buffer::Transfer<
            Channel<CH11>,
            Rx<(pac::PIO0, SM0)>,
            &'static mut [u32; SAMPLE_MEMORY / 4],
        >,
    ),
    StandBy(
        (
            Channel<CH11>,
            Rx<(pac::PIO0, SM0)>,
            &'static mut [u32; SAMPLE_MEMORY / 4],
        ),
    ),
}

pub struct Sampler {
    pio: PIO<pac::PIO0>,
    sink: Option<Sink>,
    ingest: Option<Ingest>,
    divisor: u16,
    samples: usize,
    ch_groups: [bool; 2],
}

impl Sampler {
    pub fn new(
        pio: PIO<pac::PIO0>,
        sm: UninitStateMachine<(pac::PIO0, SM0)>,
        dma: dma::Channels,
    ) -> Self {
        let mut dma_ch = dma.ch11;
        dma_ch.listen_irq0();

        let mut pio = pio;
        let mut asm = TriggerAssembler::new();
        asm.push(true, true);
        let program = pio.install(&asm.assemble_program()).unwrap();
        let (sm, rx, tx) = PIOBuilder::from_program(program).build(sm);
        let sm = sm.start();

        let samples = singleton!(: [u32; SAMPLE_MEMORY/4] = [0x00; SAMPLE_MEMORY/4]).unwrap();
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

    pub fn set_flags(&mut self, flags: u8) {
        self.ch_groups[0] = flags >> 2 & 1 == 0;
        self.ch_groups[1] = flags >> 3 & 1 == 0;
    }

    pub fn set_divisor(&mut self, divisor: u16) {
        self.divisor = divisor;
    }

    pub fn set_sample_memory(&mut self, samples: usize) {
        self.samples = samples;
    }

    pub fn start(&mut self, trigger: Trigger) {
        let (ch, rx, sample_mem) = match self.sink.take() {
            Some(Sink::StandBy(dma)) => dma,
            Some(Sink::InProgress(tx)) => tx.wait(),
            _ => unreachable!(),
        };

        match self.ingest.take() {
            Some((sm, tx)) => {
                let (sm, old) = sm.uninit(rx, tx);
                self.pio.uninstall(old);
                let program = trigger.compile();
                let program = self.pio.install(&program).unwrap();
                let (sm, rx, tx) = PIOBuilder::from_program(program)
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

    pub fn drain(&mut self, serial: &mut SerialPort<'_, UsbBus>) {
        if let Some(sink) = self.sink.take() {
            match sink {
                Sink::InProgress(mut tx) => {
                    tx.check_irq0();
                    let (ch, rx, sample_mem) = tx.wait();
                    for chunk in sample_mem.chunks(2).take(self.samples + 1).rev() {
                        let s02 = chunk[1].to_le_bytes();
                        let s13 = chunk[0].to_le_bytes();
                        match self.ch_groups {
                            [true, false] => {
                                serial.write(&[s02[0], s02[2], s13[0], s13[2]]).ok();
                            }
                            [false, true] => {
                                serial.write(&[s02[1], s02[3], s13[1], s13[3]]).ok();
                            }
                            [true, true] => {
                                serial.write(&s02).ok();
                                serial.write(&s13).ok();
                            }
                            _ => {}
                        }
                    }
                    self.sink = Some(Sink::StandBy((ch, rx, sample_mem)));
                }
                _ => unreachable!(),
            }
        }
    }
}
