//! Motion Link is a special way to restrict motion on Impulse Joint
//! 
//! Provides a way to say Joint A is restricted to subset of Joint B
//! This essentially will create a unique realtionship that you can see in gears / belts
//! 
//! Each Joint A is restricted to Joint B but this is a circular relationship, A repeated connection is possible but not encouraged
//! 
//! [`Motion Link`]: https://www.youtube.com/watch?v=jSaLy4RMnfY

use crate::math::Real;
use crate::prelude::RigidBodyHandle;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
/// Motion Link will link the movement of two joints to be connected
pub struct MotionLink {
    /// The Joint handle of the linked joint - currently only supports impusle joints
    pub body_handle: RigidBodyHandle,
    /// The ratio of motion for the connection
    pub ratio: Real,
    /// Is the second joint's axis of motion reversed? This is necessary to find the correct direction of rotation sometimes
    pub reversed: bool
}