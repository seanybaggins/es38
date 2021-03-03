#![cfg_attr(not(test), no_std)]

use core::f32::consts::PI;
use core::ops::Sub;
use defmt::Format;
use either::Either;
use embedded_hal::digital::v2::InputPin;
use embedded_time::duration::*;
use embedded_time::fixed_point::FixedPoint;
pub use rotary_encoder_hal::Direction;
use rotary_encoder_hal::Rotary;

const DEGREES_PER_REV: u16 = 360;

#[derive(Debug, Format, Copy, Clone)]
pub enum Error {
    VelocityArithmeticOverflowWouldOccur,
}

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
    /// Creates an instance of a rotary encoder assuming the rotary encoder is starting at rest.
    pub fn new(
        pin_a: A,
        pin_b: B,
        starting_angle: Angle,
        initial_time_since_epoch_milli_sec: Milliseconds<u32>,
    ) -> Self {
        let hardware = Rotary::new(pin_a, pin_b);
        let velocity = Velocity::new(
            initial_time_since_epoch_milli_sec,
            initial_time_since_epoch_milli_sec,
            starting_angle,
            starting_angle,
        );
        Encoder {
            hardware,
            angle: starting_angle,
            velocity,
        }
    }

    /// Updates memory keeping track of the angle of the encoder and what direction the encoder is moving
    /// Should only be called when there is a new angle to store with the encoder
    pub fn update(
        &mut self,
        current_time_since_epoch: Milliseconds<u32>,
    ) -> Result<Direction, Either<A::Error, B::Error>> {
        let direction = self.hardware.update()?;
        self.angle.update(direction);
        self.velocity.update(self.angle, current_time_since_epoch);
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

    /// Updates the internal state of the velocity of the
    /// rotary encoder before giving a reference
    pub fn velocity(
        &mut self,
        current_angle: Angle,
        current_time_since_epoch_milli_sec: Milliseconds<u32>,
    ) -> Velocity {
        self.velocity.final_time_since_epoch_milli_sec = current_time_since_epoch_milli_sec;
        self.velocity.final_angle = current_angle;
        let calculated_velocity = self.velocity.clone();
        self.velocity
            .update(current_angle, current_time_since_epoch_milli_sec);
        return calculated_velocity;
    }
}

// todo: Make the time requirement less restrictive
#[derive(Clone, Copy, Debug)]
pub struct Velocity {
    initial_time_since_epoch_milli_sec: Milliseconds<u32>,
    final_time_since_epoch_milli_sec: Milliseconds<u32>,
    initial_angle: Angle,
    final_angle: Angle,
}

// For nice debugging
impl defmt::Format for Velocity {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            r#"initial_time_since_epoch_milli_sec = {}
final_time_since_epoch_milli_sec = {}
initial_angle_deg = {}
final_angle_deg = {}
degrees_per_sec = {}
rad_per_sec = {}
"#,
            self.initial_time_since_epoch_milli_sec.integer(),
            self.final_time_since_epoch_milli_sec.integer(),
            self.initial_angle.degrees(),
            self.final_angle.degrees(),
            self.degrees_per_sec().unwrap_or(f32::NAN),
            self.radians_per_sec().unwrap_or(f32::NAN),
        )
    }
}

impl Velocity {
    pub fn new(
        initial_time_since_epoch_milli_sec: Milliseconds<u32>,
        final_time_since_epoch_milli_sec: Milliseconds<u32>,
        initial_angle: Angle,
        final_angle: Angle,
    ) -> Self {
        Velocity {
            initial_time_since_epoch_milli_sec,
            final_time_since_epoch_milli_sec,
            initial_angle,
            final_angle,
        }
    }

    fn update(
        &mut self,
        current_angle: Angle,
        current_time_since_epoch_milli_sec: Milliseconds<u32>,
    ) {
        self.initial_angle = self.final_angle;
        self.final_angle = current_angle;
        self.initial_time_since_epoch_milli_sec = self.final_time_since_epoch_milli_sec;
        self.final_time_since_epoch_milli_sec = current_time_since_epoch_milli_sec;
    }

    /// A helper function so there is not repetative code in radians_per_sec and degrees_per_sec
    fn angle_time_diffs(&self) -> (Angle, Result<Milliseconds<u32>, Error>) {
        let delta_angle = self.final_angle - self.initial_angle;
        if self.final_time_since_epoch_milli_sec < self.initial_time_since_epoch_milli_sec {
            return (
                delta_angle,
                Err(Error::VelocityArithmeticOverflowWouldOccur),
            );
        }
        let delta_time_milli_sec =
            self.final_time_since_epoch_milli_sec - self.initial_time_since_epoch_milli_sec;

        (delta_angle, Ok(delta_time_milli_sec))
    }

    /// This function exists so that the caller can reconstuct a velocity when a potetial
    /// arithmetic overflow is detected
    pub fn initial_time_since_epoch_milli_sec(&self) -> Milliseconds<u32> {
        self.initial_time_since_epoch_milli_sec
    }

    /// This function exists so that the caller can reconstuct a velocity when a potetial
    /// arithmetic overflow is detected
    pub fn final_time_since_epoch_milli_sec(&self) -> Milliseconds<u32> {
        self.final_time_since_epoch_milli_sec
    }

    /// This function exists so that the caller can reconstuct a velocity when a potetial
    /// arithmetic overflow is detected
    pub fn initial_angle(&self) -> Angle {
        self.initial_angle
    }

    /// This function exists so that the caller can reconstuct a velocity when a potetial
    /// arithmetic overflow is detected
    pub fn final_angle(&self) -> Angle {
        self.final_angle
    }

    pub fn radians_per_sec(&self) -> Result<f32, Error> {
        let (delta_angle, delta_time_milli_sec) = self.angle_time_diffs();
        let delta_time_milli_sec = delta_time_milli_sec?;

        Ok(delta_angle.radians() / ((*delta_time_milli_sec.integer() as f32) / 1_000.0))
    }

    pub fn degrees_per_sec(&self) -> Result<f32, Error> {
        let (delta_angle, delta_time_milli_sec) = self.angle_time_diffs();
        let delta_time_milli_sec = delta_time_milli_sec?;
        Ok(delta_angle.degrees() / ((*delta_time_milli_sec.integer() as f32) / 1_000.0))
    }
}

#[derive(Clone, Copy, Debug, Format)]
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
        let initial_time_since_epoch_milli_sec = Milliseconds(1_u32);
        let angle_180_deg = Angle::new(counts_per_rev, (counts_per_rev / 2) as i16);
        let final_time_since_epoch_milli_sec = Milliseconds(1_001_u32);
        let mut velocity = Velocity::new(
            initial_time_since_epoch_milli_sec,
            initial_time_since_epoch_milli_sec,
            initial_angle,
            initial_angle,
        );

        velocity.update(angle_180_deg, final_time_since_epoch_milli_sec);

        let velocity_degrees_per_sec = velocity.degrees_per_sec().unwrap();
        eprintln!("velocity.degrees_per_sec() = {}", velocity_degrees_per_sec);
        assert!(velocity_degrees_per_sec < 180.1 && velocity_degrees_per_sec > 179.9);

        velocity = Velocity::new(
            initial_time_since_epoch_milli_sec,
            initial_time_since_epoch_milli_sec,
            initial_angle,
            initial_angle,
        );
        velocity.update(angle_180_deg, final_time_since_epoch_milli_sec);
        let velocity_radians_per_sec = velocity.radians_per_sec().unwrap();
        eprintln!("velocity.radians_per_sec() = {}", velocity_radians_per_sec);
        assert!(velocity_radians_per_sec < (PI + 0.01) && velocity_radians_per_sec > (PI - 0.01))
    }
}
