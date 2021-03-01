#![cfg_attr(not(test), no_std)]

use core::f32::consts::PI;
use core::ops::Sub;
use either::Either;
use embedded_hal::digital::v2::InputPin;
use embedded_time::duration::*;
pub use rotary_encoder_hal::Direction;
use rotary_encoder_hal::Rotary;

const DEGREES_PER_REV: u16 = 360;

pub struct Encoder<A, B>
where
    A: InputPin,
    B: InputPin,
{
    hardware: Rotary<A, B>,
    angle: Angle,
    velocity: Velocity,
}

impl<A, B> Encoder<A, B>
where
    A: InputPin,
    B: InputPin,
{
    /// Creates an instance of a rotary encoder.
    /// The angle indicates how far the encoder starts displaced from the origin.
    pub fn new(
        pin_a: A,
        pin_b: B,
        angle: Angle,
        initial_time_since_epoch_milli_sec: Milliseconds<u32>,
    ) -> Self {
        let hardware = Rotary::new(pin_a, pin_b);
        let velocity = Velocity::new(initial_time_since_epoch_milli_sec, angle);
        Encoder {
            hardware,
            angle,
            velocity,
        }
    }

    /// Updates memory keeping track of the angle of the encoder and what direction the encoder is moving
    pub fn update(
        &mut self,
        time_since_epoch_milli_sec: Milliseconds<u32>,
    ) -> Result<Direction, Either<A::Error, B::Error>> {
        let direction = self.hardware.update()?;
        self.angle.update(direction);
        self.velocity.update(self.angle, time_since_epoch_milli_sec);
        Ok(direction)
    }

    /// Returns a mutible reference to the underlying hardware so one
    /// can clear the interrupt pending bits of the rotary_encoder.
    pub fn hardware(&mut self) -> &mut Rotary<A, B> {
        &mut self.hardware
    }

    /// Gets the current angle of the rotary encoder
    pub fn angle(&self) -> &Angle {
        &self.angle
    }

    /// Gets the current velocity of the rotary encoder
    pub fn velocity(&self) -> &Velocity {
        &self.velocity
    }
}

// todo: Make the time requirement less restrictive
#[derive(Clone, Copy, Debug)]
pub struct Velocity {
    pub initial_time_since_epoch_milli_sec: Milliseconds<u32>,
    pub final_time_since_epoch_milli_sec: Milliseconds<u32>,
    pub initial_angle: Angle,
    pub final_angle: Angle,
}

impl Velocity {
    /// Creates a velocity instance assumming that the object is starting at rest
    pub fn new(first_time_since_epoch_millisec: Milliseconds<u32>, angle: Angle) -> Self {
        Velocity {
            initial_time_since_epoch_milli_sec: first_time_since_epoch_millisec,
            final_time_since_epoch_milli_sec: first_time_since_epoch_millisec,
            initial_angle: angle,
            final_angle: angle,
        }
    }

    pub fn radians_per_sec(&self) -> f32 {
        let delta_angle = self.final_angle - self.initial_angle;
        let delta_time_milli_sec =
            self.final_time_since_epoch_milli_sec - self.initial_time_since_epoch_milli_sec;
        let delta_time_sec: Seconds<u32> = delta_time_milli_sec.into();
        delta_angle.radians() / (*delta_time_sec.integer() as f32)
    }

    pub fn degrees_per_sec(&self) -> f32 {
        let delta_angle = self.final_angle - self.initial_angle;
        let delta_time_milli_sec =
            self.final_time_since_epoch_milli_sec - self.initial_time_since_epoch_milli_sec;
        let delta_time_sec: Seconds<u32> = delta_time_milli_sec.into();

        delta_angle.degrees() / (*delta_time_sec.integer() as f32)
    }

    /// Updates the velocity given the current angle and instant the angle was captured
    fn update(&mut self, angle: Angle, time_since_epoch_milli_sec: Milliseconds<u32>) {
        self.initial_angle = self.final_angle;
        self.final_angle = angle;
        self.initial_time_since_epoch_milli_sec = self.final_time_since_epoch_milli_sec;
        self.final_time_since_epoch_milli_sec = time_since_epoch_milli_sec;
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Angle {
    /// counts of the rotary encoder
    counts: i16,
    /// How many counts there are for a the rotary_encoder
    counts_per_rev: u16,
}

impl Sub for Angle {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        // todo: remove this assertion and return a result instead
        assert!(self.counts_per_rev == other.counts_per_rev);

        Self {
            counts: self.counts - other.counts,
            counts_per_rev: self.counts_per_rev,
        }
    }
}
impl Angle {
    /// Creates a angle type given the maximum counts per revolutions and how far, in counts,
    /// the the physical location of the rotary encoders position is displaced from the origin.
    pub fn new(counts_per_rev: u16, origin_offset_counts: i16) -> Self {
        Angle {
            counts: origin_offset_counts,
            counts_per_rev,
        }
    }

    /// Increments or decrements the counter depending on the direction
    fn update(&mut self, direction: Direction) {
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

    #[test]
    fn correct_velocity_calculations() {
        let counts_per_rev = 2400;
        let origin_offset = 0;
        let initial_angle = Angle::new(counts_per_rev, origin_offset);
        let initial_time_since_epoch_milli_sec = Milliseconds(0_u32);
        let mut velocity = Velocity::new(initial_time_since_epoch_milli_sec, initial_angle);

        let angle_180_deg = Angle::new(counts_per_rev, (counts_per_rev / 2) as i16);
        let final_time_since_epoch_milli_sec = Milliseconds(1000_u32);
        velocity.update(angle_180_deg, final_time_since_epoch_milli_sec);

        eprintln!(
            "velocity.degrees_per_sec() = {}",
            velocity.degrees_per_sec()
        );
        assert!(velocity.degrees_per_sec() < 180.1 && velocity.degrees_per_sec() > 179.9);

        eprintln!(
            "velocity.radians_per_sec() = {}",
            velocity.radians_per_sec()
        );
        assert!(
            velocity.radians_per_sec() < (PI + 0.01) && velocity.radians_per_sec() > (PI - 0.01)
        )
    }
}
