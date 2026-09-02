//! FFI bindings for the JAKA SDK C API.
//!
//! Types and function signatures correspond to the headers under
//! sdk/Linux/c&c++/inc_of_c/:
//! - jktypes.h: base types and structs
//! - jakaAPI.h: C API functions
//! - jkerr.h:   error codes
//!
//! The shared library is linked by build.rs from
//! sdk/Linux/c&c++/x86_64-linux-gnu/shared/libjakaAPI.so.

use std::ffi::{c_char, c_uint, c_void};

/// Robot control handle (jktypes.h: JKHD)
pub type JKHD = i32;
/// SDK BOOL (jktypes.h: BOOL)
pub type BOOL = i32;
/// SDK error code (jktypes.h: errno_t)
#[allow(non_camel_case_types)]
pub type errno_t = i32;

pub const ERR_SUCC: errno_t = 0;

/// Joint position in radians (jktypes.h: JointValue)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct JointValue {
    pub j_val: [f64; 6],
}

impl JointValue {
    pub fn zero() -> Self {
        Self { j_val: [0.0; 6] }
    }
}

/// Cartesian translation in mm (jktypes.h: CartesianTran)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CartesianTran {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Cartesian orientation as RPY in radians (jktypes.h: Rpy)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Rpy {
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
}

/// Cartesian pose, translation in mm and orientation in radians (jktypes.h: CartesianPose)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CartesianPose {
    pub tran: CartesianTran,
    pub rpy: Rpy,
}

impl CartesianPose {
    pub fn zero() -> Self {
        Self::default()
    }
}

/// Simplified robot state (jktypes.h: RobotState)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RobotState {
    pub estoped: BOOL,
    pub powered_on: BOOL,
    pub servo_enabled: BOOL,
}

/// DH parameters (jktypes.h: DHParam)
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct DHParam {
    pub alpha: [f64; 6],
    pub a: [f64; 6],
    pub d: [f64; 6],
    pub joint_homeoff: [f64; 6],
}

/// Motion mode (jktypes.h: MoveMode)
#[repr(i32)]
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)] // Continue is only used by the kept single-axis jog API
pub enum MoveMode {
    Abs = 0,
    Incr = 1,
    Continue = 2,
}

/// Coordinate system (jktypes.h: CoordType)
#[repr(i32)]
#[allow(dead_code)] // Joint/Tool are not used yet but kept to mirror the header
#[derive(Clone, Copy, Debug)]
pub enum CoordType {
    Base = 0,
    Joint = 1,
    Tool = 2,
}

/// Optional motion parameters, always null in this project (jktypes.h: OptionalCond)
#[repr(C)]
pub struct OptionalCond {
    _private: [u8; 0],
}

#[link(name = "jakaAPI")]
unsafe extern "C" {
    /// Create a connection handle
    #[link_name = "jk_safe_create_handler"]
    pub fn create_handler(ip: *const c_char, handle: *mut JKHD) -> errno_t;
    /// Destroy the handle and disconnect. The SDK header spells it "destory"
    #[link_name = "jk_safe_destory_handler"]
    pub fn destory_handler(handle: *const JKHD) -> errno_t;
    #[link_name = "jk_safe_power_on"]
    pub fn power_on(handle: *const JKHD) -> errno_t;
    #[link_name = "jk_safe_power_off"]
    pub fn power_off(handle: *const JKHD) -> errno_t;
    #[link_name = "jk_safe_enable_robot"]
    pub fn enable_robot(handle: *const JKHD) -> errno_t;
    #[link_name = "jk_safe_disable_robot"]
    pub fn disable_robot(handle: *const JKHD) -> errno_t;
    #[link_name = "jk_safe_get_robot_state"]
    pub fn get_robot_state(handle: *const JKHD, state: *mut RobotState) -> errno_t;
    #[link_name = "jk_safe_get_dh_param"]
    pub fn get_dh_param(handle: *const JKHD, dh_param: *mut DHParam) -> errno_t;
    #[link_name = "jk_safe_get_joint_position"]
    pub fn get_joint_position(handle: *const JKHD, pos: *mut JointValue) -> errno_t;
    #[link_name = "jk_safe_get_tcp_position"]
    pub fn get_tcp_position(handle: *const JKHD, tcp_position: *mut CartesianPose) -> errno_t;
    /// Inverse kinematics with a reference joint pose to disambiguate solutions
    #[link_name = "jk_safe_kine_inverse"]
    pub fn kine_inverse(
        handle: *const JKHD,
        ref_pos: *const JointValue,
        cartesian_pose: *const CartesianPose,
        joint_pos: *mut JointValue,
    ) -> errno_t;
    /// Forward kinematics
    #[link_name = "jk_safe_kine_forward"]
    pub fn kine_forward(
        handle: *const JKHD,
        joint_pos: *const JointValue,
        cartesian_pose: *mut CartesianPose,
    ) -> errno_t;
    #[link_name = "jk_safe_joint_move_extend"]
    pub fn joint_move_extend(
        handle: *const JKHD,
        joint_pos: *const JointValue,
        move_mode: MoveMode,
        is_block: BOOL,
        speed: f64,
        acc: f64,
        tol: f64,
        option_cond: *const OptionalCond,
    ) -> errno_t;
    #[link_name = "jk_safe_linear_move_extend"]
    pub fn linear_move_extend(
        handle: *const JKHD,
        end_pos: *const CartesianPose,
        move_mode: MoveMode,
        is_block: BOOL,
        speed: f64,
        acc: f64,
        tol: f64,
        option_cond: *const OptionalCond,
    ) -> errno_t;
    /// Stop all ongoing movements of the cobot
    #[link_name = "jk_safe_motion_abort"]
    pub fn motion_abort(handle: *const JKHD) -> errno_t;
    /// Continuous velocity control of one axis. Kept because a jog call
    /// preempts the previous motion, so the serve protocol uses servo mode
    /// instead and this API is no longer called
    #[allow(dead_code)]
    #[link_name = "jk_safe_jog"]
    pub fn jog(
        handle: *const JKHD,
        aj_num: i32,
        move_mode: MoveMode,
        coord_type: CoordType,
        vel_cmd: f64,
        pos_cmd: f64,
    ) -> errno_t;
    /// Stop the ongoing jog movement of one axis
    #[allow(dead_code)]
    #[link_name = "jk_safe_jog_stop"]
    pub fn jog_stop(handle: *const JKHD, num: i32) -> errno_t;
    /// Enter or leave servo mode, the smooth multi-axis control mode
    #[link_name = "jk_safe_servo_move_enable"]
    pub fn servo_move_enable(handle: *const JKHD, enable: BOOL) -> errno_t;
    /// One servo interpolation cycle of Cartesian motion, the pose is an
    /// absolute target in ABS mode or a delta in INCR mode, translations in
    /// mm and rotations in radians. step_num times 8 ms is the cycle period
    #[link_name = "jk_safe_servo_p"]
    pub fn servo_p(
        handle: *const JKHD,
        pose: *const CartesianPose,
        move_mode: MoveMode,
        step_num: c_uint,
    ) -> errno_t;
    #[link_name = "jk_safe_is_in_estop"]
    pub fn is_in_estop(handle: *const JKHD, in_estop: *mut BOOL) -> errno_t;
    #[link_name = "jk_safe_clear_error"]
    pub fn clear_error(handle: *const JKHD) -> errno_t;
}

// POSIX file descriptor helpers used to keep SDK printf output off stdout
#[cfg(unix)]
unsafe extern "C" {
    fn dup(oldfd: i32) -> i32;
    fn dup2(oldfd: i32, newfd: i32) -> i32;
    fn write(fd: i32, buf: *const c_void, count: usize) -> isize;
}

// MSVCRT file descriptor helpers, the Windows counterpart of the above
#[cfg(windows)]
unsafe extern "system" {
    fn _dup(oldfd: i32) -> i32;
    fn _dup2(oldfd: i32, newfd: i32) -> i32;
    fn _write(fd: i32, buf: *const c_void, count: u32) -> i32;
}

/// Redirect stdout to stderr and return the original stdout file descriptor.
/// The SDK prints connection progress with printf at arbitrary times, so fd 1
/// stays redirected for the whole session. Machine-readable output is written
/// through the returned fd instead.
pub fn redirect_stdout_to_stderr() -> i32 {
    #[cfg(unix)]
    let saved = unsafe { dup(1) };
    #[cfg(windows)]
    let saved = unsafe { _dup(1) };
    #[cfg(unix)]
    unsafe {
        dup2(2, 1)
    };
    #[cfg(windows)]
    unsafe {
        _dup2(2, 1)
    };
    saved
}

/// Write a string to a file descriptor, retrying on partial writes
pub fn write_to_fd(fd: i32, s: &str) {
    let bytes = s.as_bytes();
    let mut written = 0;
    while written < bytes.len() {
        #[cfg(unix)]
        let n = unsafe {
            write(
                fd,
                bytes[written..].as_ptr() as *const c_void,
                bytes.len() - written,
            )
        };
        #[cfg(windows)]
        let n = unsafe {
            _write(
                fd,
                bytes[written..].as_ptr() as *const c_void,
                (bytes.len() - written) as u32,
            )
        };
        if n <= 0 {
            break;
        }
        written += n as usize;
    }
}

/// Human readable error names (jkerr.h)
pub fn err_name(code: errno_t) -> &'static str {
    match code {
        0 => "Success",
        -1 => "Invalid handler",
        -2 => "Invalid parameter",
        -3 => "Connection failed",
        -5 => "E-stop pressed",
        -6 => "Not powered on",
        -7 => "Not enabled",
        -10 => "Program running",
        -12 => "Motion abnormal",
        -21 => "Emergency stop",
        -22 => "Soft limit reached",
        -41 => "Joint move failed",
        -42 => "Circular move failed",
        -999 => "SDK exception",
        _ => "Unknown error",
    }
}

/// Check an SDK return code and turn failures into a descriptive error
pub fn check(label: &str, code: errno_t) -> Result<(), String> {
    if code == ERR_SUCC {
        Ok(())
    } else {
        Err(format!(
            "{label} failed with error code {code}: {}",
            err_name(code)
        ))
    }
}
