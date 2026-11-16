//! The Roomba driver's protocol and logic, kept free of ROS types so it can be unit tested - except
//! `roomba`, which owns the Roomba's serial port. The `roomba_driver` binary (src/bin) connects
//! them to ROS. `oi` and `battery` are also used by the robot stack's 2D digital twin
//! (roomba-jetson-stack, roomba_rs twin2d) to emulate the Roomba's Open Interface and battery.
pub mod battery;
pub mod driver;
pub mod kinematics;
pub mod oi;
pub mod roomba;
