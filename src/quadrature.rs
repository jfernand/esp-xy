//! Quadrature (A/B) decoder built on the PCNT (pulse counter) peripheral.
//!
//! Wiring channel 0 to (ctrl = A, edge = B) and channel 1 to (ctrl = B, edge = A) within a
//! single PCNT unit -- with the control signal's level flipping the edge channel's count
//! direction -- gives full 4x quadrature decoding entirely in hardware: no CPU time spent
//! polling GPIOs or servicing edge interrupts. See:
//! <https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/pcnt.html>

use esp_hal::gpio::Input;
use esp_hal::pcnt::channel::{CtrlMode, EdgeMode};
use esp_hal::pcnt::unit::Unit;

/// Quadrature decoder built on a single PCNT unit.
///
/// The hardware counter tracks position directly; call [`Self::count`] whenever you need
/// the current value. It wraps at the counter's 16-bit limits (`i16::MIN`/`i16::MAX`)
/// unless you use [`Self::unit`] to set low/high limits plus an interrupt handler that
/// accumulates past them.
pub struct QuadratureDecoder<'d, const UNIT: usize> {
    unit: Unit<'d, UNIT>,
    _pin_a: Input<'d>,
    _pin_b: Input<'d>,
    position: i64,
    last_raw: i16,
}

impl<'d, const UNIT: usize> QuadratureDecoder<'d, UNIT> {
    /// Configures `unit` to decode the quadrature signals on `pin_a`/`pin_b`.
    ///
    /// Set the pins' pull resistors (typically `Pull::Up`) before passing them in --
    /// most encoders are open-drain and need it.
    pub fn new(unit: Unit<'d, UNIT>, pin_a: Input<'d>, pin_b: Input<'d>) -> Self {
        let sig_a = pin_a.peripheral_input();
        let sig_b = pin_b.peripheral_input();

        let ch0 = &unit.channel0;
        ch0.set_ctrl_signal(sig_a.clone());
        ch0.set_edge_signal(sig_b.clone());
        ch0.set_ctrl_mode(CtrlMode::Reverse, CtrlMode::Keep);
        ch0.set_input_mode(EdgeMode::Increment, EdgeMode::Decrement);

        let ch1 = &unit.channel1;
        ch1.set_ctrl_signal(sig_b);
        ch1.set_edge_signal(sig_a);
        ch1.set_ctrl_mode(CtrlMode::Reverse, CtrlMode::Keep);
        ch1.set_input_mode(EdgeMode::Decrement, EdgeMode::Increment);

        unit.clear();
        unit.resume();

        Self {
            unit,
            _pin_a: pin_a,
            _pin_b: pin_b,
            position: 0,
            last_raw: 0,
        }
    }

    /// Raw hardware count: increments/decrements on every edge of either A or B (4x
    /// resolution), positive for one rotation direction and negative for the other. Wraps at
    /// the counter's 16-bit limits -- for anything that runs longer than a full wrap, call
    /// [`Self::update`] instead.
    pub fn count(&self) -> i16 {
        self.unit.value()
    }

    /// Folds the latest raw count into an extended, non-wrapping position and returns it.
    ///
    /// Call this periodically (e.g. from a control loop tick) rather than reading [`Self::count`]
    /// directly: it takes the wrapping difference from the last call, which recovers the true
    /// delta as long as the true movement between calls never reaches the 16-bit counter's
    /// half-range (65536 counts) -- true for any realistic polling rate, since that would mean
    /// hundreds of thousands of counts per tick.
    pub fn update(&mut self) -> i64 {
        let raw = self.unit.value();
        let delta = raw.wrapping_sub(self.last_raw);
        self.last_raw = raw;
        self.position += delta as i64;
        self.position
    }

    /// The extended position as of the last [`Self::update`] call, without polling the
    /// hardware counter again.
    pub fn position(&self) -> i64 {
        self.position
    }

    /// Resets both the hardware counter and the extended position to zero.
    pub fn reset(&mut self) {
        self.unit.clear();
        self.position = 0;
        self.last_raw = 0;
    }

    /// The underlying PCNT unit, e.g. to configure limits/interrupts for extending past
    /// the 16-bit counter range or to add a glitch filter via `set_filter`.
    pub fn unit(&self) -> &Unit<'d, UNIT> {
        &self.unit
    }
}
