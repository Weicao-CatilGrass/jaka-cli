//! DualSense gamepad controller for the JAKA arm.
//!
//! This binary is a pure protocol layer: it reads a DualSense gamepad and
//! streams move and rotate commands to the jaka-cli serve endpoint over TCP.
//! It never links the JAKA SDK.
//!
//! Placeholder, the gamepad mapping is not implemented yet.

fn main() {
    print!("{}", include_str!("jaka_ds5_control_help.txt"));
}
