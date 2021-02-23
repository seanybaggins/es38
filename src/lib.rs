#![cfg_attr(not(test), no_std)]

use core::f32::consts::PI;
use either::Either;
use embedded_hal::digital::v2::InputPin;
use rotary_encoder_hal::{Rotary};
pub use rotary_encoder_hal::Direction;
const DEGREES_PER_REV: u16 = 360;

pub struct Encoder<A, B>
where
    A: InputPin,
    B: InputPin,
{
    hardware: Rotary<A, B>,
    angle: Angle,
}

impl<A, B> Encoder<A, B>
where
    A: InputPin,
    B: InputPin,
{
    /// Creates an instance of a rotary encoder.
    /// The `origin_offset` indicates how far the initial rotary encoders starting position is from the origin
    pub fn new(pin_a: A, pin_b: B, counts_per_rev: u16, origin_offset: i16) -> Self {
        let hardware = Rotary::new(pin_a, pin_b);
        let angle = Angle::new(counts_per_rev, origin_offset);
        Encoder { hardware, angle }
    }

    /// Updates memory keeping track of the angle of the encoder and what direction the encoder is moving
    pub fn update(&mut self) -> Result<Direction, Either<A::Error, B::Error>> {
        let direction = self.hardware.update()?;
        self.angle.update(direction);

        Ok(direction)
    }

    pub fn hardware(&mut self) -> &mut Rotary<A, B> {
        &mut self.hardware
    }

    pub fn angle(&mut self) -> &mut Angle {
        &mut self.angle
    }
}

pub struct Angle {
    /// counts of the rotary encoder
    counts: i16,

    counts_per_rev: u16,
}

impl Angle {
    /// Creates a angle type given the maximum counts per revolutions and how far, in counts,
    /// the the physical location of the rotary encoders position is displaced from the origin.
    pub(self) fn new(counts_per_rev: u16, origin_offset_counts: i16) -> Self {
        Angle {
            counts: origin_offset_counts,
            counts_per_rev,
        }
    }

    /// Increments or decrements the counter depending on the direction
    pub(self) fn update(&mut self, direction: Direction) {
        match direction {
            Direction::CounterClockwise => self.counts += 1,
            Direction::Clockwise => self.counts -= 1,
            Direction::None => (),
        };
    }

    /// Gets the angle of the encoder in radians
    pub fn radians(&self) -> f32 {
        self.degrees() * PI / 180.0
    }

    /// Gets the angle of the encoder in degrees.
    pub fn degrees(&self) -> f32 {
        let counts = self.counts as f32;
        let degrees_per_rev = DEGREES_PER_REV as f32;
        let counts_per_rev = self.counts_per_rev as f32;

        counts * degrees_per_rev / counts_per_rev
    }

    pub fn counts(&self) -> i16 {
        self.counts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn correct_angle_rotated_back_to_origin_deg() {
        let counts_per_rev = 600;
        let origin_offset = 300;
        let mut angle = Angle::new(counts_per_rev, origin_offset);
        let direction = Direction::Clockwise;

        for _ in 0..300 {
            angle.update(direction);
        }

        assert!(angle.degrees() > -0.1 && angle.degrees() < 0.1);
    }
}
