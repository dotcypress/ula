use crate::*;

/// Type alias for the PIO assembler with a 32-bit instruction width.
pub type TriggerAssembler = pio::Assembler<32>;

/// Type alias for the compiled PIO trigger program.
pub type TriggerProgram = pio::Program<32>;

/// Struct representing a single trigger stage with mask, pattern, and delay.
#[derive(Default, Clone, Copy)]
pub struct TriggerStage {
    /// Bitmask for the trigger condition.
    mask: u32,
    /// Pattern to match for triggering.
    pattern: u32,
    /// Delay before the trigger is activated.
    delay: u32,
}

/// Struct representing the trigger configuration with multiple stages.
#[derive(Default, Clone, Copy)]
pub struct Trigger {
    /// Array of trigger stages.
    stages: [TriggerStage; 4],
}

impl Trigger {
    /// Sets the mask for a specific trigger stage.
    ///
    /// # Arguments
    ///
    /// * `stage` - Index of the trigger stage (0-3).
    /// * `mask` - Bitmask to set for the trigger.
    pub fn set_mask(&mut self, stage: usize, mask: u32) {
        self.stages[stage].mask = mask;
    }

    /// Sets the pattern for a specific trigger stage.
    ///
    /// # Arguments
    ///
    /// * `stage` - Index of the trigger stage (0-3).
    /// * `pattern` - Pattern to set for the trigger.
    pub fn set_pattern(&mut self, stage: usize, pattern: u32) {
        self.stages[stage].pattern = pattern;
    }

    /// Sets the delay for a specific trigger stage.
    ///
    /// # Arguments
    ///
    /// * `stage` - Index of the trigger stage (0-3).
    /// * `delay` - Delay to set for the trigger.
    pub fn set_delay(&mut self, stage: usize, delay: u32) {
        self.stages[stage].delay = delay;
    }

    /// Compiles the trigger configuration into a PIO program.
    ///
    /// This method assembles the trigger logic based on the configured stages.
    ///
    /// # Returns
    ///
    /// A compiled `TriggerProgram` ready to be installed into PIO.
    pub fn compile(&self) -> TriggerProgram {
        let mut asm = TriggerAssembler::new();
        let mut wrap_target = asm.label();
        let mut wrap_source = asm.label();

        // Iterate over each trigger stage that has a non-zero mask.
        for TriggerStage {
            mut mask,
            mut pattern,
            delay: _,
        } in self.stages.iter().filter(|s| s.mask != 0)
        {
            let mut stage_label = asm.label();
            asm.bind(&mut stage_label);

            // Move bits from PINS to OSR with bit reversal.
            asm.mov(
                pio::MovDestination::OSR,
                pio::MovOperation::BitReverse,
                pio::MovSource::PINS,
            );

            loop {
                match mask.trailing_zeros() {
                    0 => {}
                    32 => break, // Exit loop if no more bits are set.
                    zeros => {
                        // Skip the trailing zeros by outputting NULL bits.
                        asm.out(pio::OutDestination::NULL, zeros as _);
                        // Shift the pattern and mask to process the next set of bits.
                        pattern >>= zeros;
                        mask >>= zeros;
                    }
                };
                match mask.trailing_ones() {
                    0 => {}
                    1 => {
                        // If the next bit is a single one, handle it as a condition.
                        let cond = if pattern & 1 == 1 {
                            pio::JmpCondition::XIsZero
                        } else {
                            pio::JmpCondition::XDecNonZero
                        };
                        asm.out(pio::OutDestination::X, 1);
                        asm.jmp(cond, &mut stage_label);
                        // Shift the pattern and mask after processing.
                        pattern >>= 1;
                        mask >>= 1;
                    }
                    ones => {
                        // Handle multiple consecutive ones, up to 5 bits.
                        let bits = ones.min(5);
                        let val = pattern & ((1 << bits) - 1);
                        asm.set(pio::SetDestination::Y, val as _);
                        asm.out(pio::OutDestination::X, bits as _);
                        asm.jmp(pio::JmpCondition::XNotEqualY, &mut stage_label);
                        // Shift the pattern and mask after processing.
                        pattern >>= bits;
                        mask >>= bits;
                    }
                };
            }
        }

        // Bind the wrap target and source labels.
        asm.bind(&mut wrap_target);
        asm.r#in(pio::InSource::PINS, PROBES as _);
        asm.bind(&mut wrap_source);

        // Assemble the program with wrap points and set the origin.
        asm.assemble_with_wrap(wrap_source, wrap_target)
            .set_origin(Some(0))
    }
}
