use crate::*;

pub type TriggerAssembler = pio::Assembler<32>;
pub type TriggerProgram = pio::Program<32>;

#[derive(Default, Clone, Copy)]
pub struct TriggerStage {
    mask: u32,
    pattern: u32,
    delay: u32,
}

#[derive(Default, Clone, Copy)]
pub struct Trigger {
    stages: [TriggerStage; 4],
}

impl Trigger {
    pub fn set_mask(&mut self, stage: usize, mask: u32) {
        self.stages[stage].mask = mask;
    }

    pub fn set_pattern(&mut self, stage: usize, pattern: u32) {
        self.stages[stage].pattern = pattern;
    }

    pub fn set_delay(&mut self, stage: usize, delay: u32) {
        self.stages[stage].delay = delay;
    }

    pub fn compile(&self) -> TriggerProgram {
        let mut asm = TriggerAssembler::new();
        let mut wrap_target = asm.label();
        let mut wrap_source = asm.label();

        for stage in self.stages {
            let TriggerStage {
                mut mask,
                mut pattern,
                delay: _,
            } = stage;

            if mask == 0 {
                continue;
            }

            let mut stage_label = asm.label();
            asm.bind(&mut stage_label);

            #[cfg(not(feature = "serial_trigger"))]
            {
                asm.mov(
                    pio::MovDestination::OSR,
                    pio::MovOperation::BitReverse,
                    pio::MovSource::PINS,
                );

                loop {
                    match mask.trailing_zeros() {
                        0 => {}
                        32 => break,
                        zeros => {
                            asm.out(pio::OutDestination::NULL, zeros as _);
                            pattern >>= zeros;
                            mask >>= zeros;
                        }
                    };
                    match mask.trailing_ones() {
                        0 => {}
                        1 => {
                            let cond = if pattern & 1 == 1 {
                                pio::JmpCondition::XIsZero
                            } else {
                                pio::JmpCondition::XDecNonZero
                            };
                            asm.out(pio::OutDestination::X, 1);
                            asm.jmp(cond, &mut stage_label);
                            pattern >>= 1;
                            mask >>= 1;
                        }
                        ones => {
                            let bits = ones.min(5);
                            let val = pattern & ((1 << bits) - 1);

                            asm.set(pio::SetDestination::Y, val as _);
                            asm.out(pio::OutDestination::X, bits as _);
                            asm.jmp(pio::JmpCondition::XNotEqualY, &mut stage_label);
                            pattern >>= bits;
                            mask >>= bits;
                        }
                    };
                }
            }

            #[cfg(feature = "serial_trigger")]
            {
                for probe in 0..PROBES {
                    if mask & 1 == 1 {
                        asm.wait(pattern as u8 & 1, pio::WaitSource::PIN, probe as _, false)
                    }
                    pattern >>= 1;
                    mask >>= 1;
                }
            }
        }

        asm.bind(&mut wrap_target);
        asm.r#in(pio::InSource::PINS, PROBES as _);
        asm.bind(&mut wrap_source);

        asm.assemble_with_wrap(wrap_source, wrap_target)
            .set_origin(Some(0))
    }
}
