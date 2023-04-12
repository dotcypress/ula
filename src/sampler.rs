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
            &'static mut [u32; SAMPLE_MEMORY / 2],
        >,
    ),
    StandBy(
        (
            Channel<CH11>,
            Rx<(pac::PIO0, SM0)>,
            &'static mut [u32; SAMPLE_MEMORY / 2],
        ),
    ),
}

pub struct Sampler {
    pio: PIO<pac::PIO0>,
    sink: Option<Sink>,
    ingest: Option<Ingest>,
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
        let mut asm = pio::Assembler::<32>::new();
        asm.push(true, true);
        let bytecode = asm.assemble_program();
        let program = pio.install(&bytecode).unwrap();
        let (sm, rx, tx) = hal::pio::PIOBuilder::from_program(program).build(sm);
        let sm = sm.start();

        let samples = singleton!(: [u32; SAMPLE_MEMORY/2] = [0x00; SAMPLE_MEMORY/2]).unwrap();
        let sink = Sink::StandBy((dma_ch, rx, samples));

        Self {
            pio,
            ingest: Some((sm, tx)),
            sink: Some(sink),
        }
    }

    pub fn start(&mut self, divisor: u16, trigger: Trigger) {
        let (ch, rx, samples) = match self.sink.take() {
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
                let (sm, rx, tx) = hal::pio::PIOBuilder::from_program(program)
                    .clock_divisor_fixed_point(divisor + 1, 0)
                    .autopush(true)
                    .in_pin_base(PIN_BASE as _)
                    .build(sm);
                let transfer = single_buffer::Config::new(ch, rx, samples);
                self.sink = Some(Sink::InProgress(transfer.start()));
                self.ingest = Some((sm.start(), tx));
            }
            _ => unreachable!(),
        }
    }

    pub fn drain(&mut self, serial: &mut SerialPort<'_, UsbBus>, amount: usize, flags: u8) {
        if let Some(sink) = self.sink.take() {
            match sink {
                Sink::InProgress(mut tx) => {
                    tx.check_irq0();
                    let (ch, rx, samples) = tx.wait();

                    let group0 = flags >> 2 & 1 == 0;
                    let group1 = flags >> 3 & 1 == 0;

                    for chunk in samples.chunks(2).take(amount + 1).rev() {
                        let s02 = chunk[1].to_le_bytes();
                        let s13 = chunk[0].to_le_bytes();
                        match (group0, group1) {
                            (true, false) => {
                                serial.write(&[s02[0], s02[2], s13[0], s13[2]]).ok();
                            }
                            (false, true) => {
                                serial.write(&[s02[1], s02[3], s13[1], s13[3]]).ok();
                            }
                            (true, true) => {
                                serial.write(&s02).ok();
                                serial.write(&s13).ok();
                            }
                            _ => {}
                        }
                    }

                    self.sink = Some(Sink::StandBy((ch, rx, samples)));
                }
                _ => unreachable!(),
            }
        }
    }
}
