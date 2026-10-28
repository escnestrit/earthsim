//! `phys` — dependency-free vehicle physics for the BeamNG.Earth prototype.
pub mod controls;
pub mod drivetrain;
pub mod math;
pub mod softbody;
pub mod softcar;
pub mod tire;
pub mod vehicle;
pub mod world;

pub use controls::{Analog, Driver, Keys};
pub use drivetrain::{Drivetrain, DrivetrainParams};
pub use math::{v3, Quat, V3};
pub use softbody::SoftBody;
pub use softcar::SoftVehicle;
pub use vehicle::{Input, Telemetry, Vehicle, VehicleParams, Wheel};
pub use world::{Contact, FlatGround, FnGround, Ground, GroundSample, Obstacles, SlopeGround, StaticWorld};
