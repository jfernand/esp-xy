//! Step/direction pulse generator for stepper drivers, built on MCPWM.
//!
//! MCPWM free-runs a square wave at whatever frequency [`StepGenerator::set_rate`] last
//! requested -- there's no hardware "emit exactly N pulses then stop" mode on this peripheral.
//! Callers that need an exact pulse count (e.g. tracking an absolute position) should
//! recompute their target from its own source of truth on every call and derive the rate
//! needed to close the gap by the next call, rather than integrating pulses open-loop: any one
//! call's rounding error is then corrected by the next instead of accumulating.

use esp_hal::gpio::{Level, Output};
use esp_hal::mcpwm::operator::PwmPin;
use esp_hal::mcpwm::timer::{PwmWorkingMode, Timer};
use esp_hal::mcpwm::{FrequencyError, PeripheralClockConfig, PwmPeripheral};
use esp_hal::time::Rate;

/// Rotation direction for [`StepGenerator::set_rate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
}

/// Drives a step/dir stepper input: a PWM square wave on the step pin (via MCPWM) plus a plain
/// GPIO level on the dir pin.
pub struct StepGenerator<'d, PWM, const TIM: u8, const OP: u8> {
    clock_cfg: PeripheralClockConfig,
    timer: Timer<TIM, PWM>,
    _step_pin: PwmPin<'d, PWM, OP, true>,
    dir_pin: Output<'d>,
    period: u16,
}

impl<'d, PWM: PwmPeripheral, const TIM: u8, const OP: u8> StepGenerator<'d, PWM, TIM, OP> {
    /// `timer` and `step_pin` must already be wired together (`operator.set_timer(&timer)`)
    /// on the same MCPWM instance `clock_cfg` was derived from. `period` trades off pulse-rate
    /// resolution against range -- see
    /// [`PeripheralClockConfig::timer_clock_with_frequency`].
    pub fn new(
        clock_cfg: PeripheralClockConfig,
        mut timer: Timer<TIM, PWM>,
        mut step_pin: PwmPin<'d, PWM, OP, true>,
        dir_pin: Output<'d>,
        period: u16,
    ) -> Self {
        timer.stop();
        step_pin.set_timestamp(period / 2);
        Self {
            clock_cfg,
            timer,
            _step_pin: step_pin,
            dir_pin,
            period,
        }
    }

    /// Sets direction and step rate. `steps_per_sec == 0` stops the pulse train (the dir pin
    /// is still updated).
    pub fn set_rate(
        &mut self,
        steps_per_sec: u32,
        direction: Direction,
    ) -> Result<(), FrequencyError> {
        self.dir_pin.set_level(match direction {
            Direction::Forward => Level::High,
            Direction::Reverse => Level::Low,
        });

        if steps_per_sec == 0 {
            self.timer.stop();
            return Ok(());
        }

        let cfg = self.clock_cfg.timer_clock_with_frequency(
            self.period,
            PwmWorkingMode::Increase,
            Rate::from_hz(steps_per_sec),
        )?;
        self.timer.start(cfg);
        Ok(())
    }

    /// Stops the pulse train immediately, leaving direction unchanged.
    pub fn stop(&mut self) {
        self.timer.stop();
    }
}
