//! JAKA robotic arm driver.
//!
//! Usage is documented in src/help.txt and printed by `jaka-cli --help`.
//! The log level is controlled by the RUST_LOG environment variable and defaults to info.
//!
//! The runtime is tokio. Blocking SDK calls run on blocking tasks so the async
//! runtime stays responsive to the Ctrl+C signal, which aborts any ongoing motion.

mod binding;
mod cli;

use std::ffi::CString;
use std::path::Path;
use std::process::ExitCode;
use std::time::Duration;

use binding::{
    BOOL, CartesianPose, CartesianTran, DHParam, JKHD, JointValue, MoveMode, OptionalCond,
    RobotState, Rpy, check, errno_t,
};
use clap::Parser;
use cli::{Cli, Command};
use log::{error, info, warn};
use serde::{Deserialize, Serialize};

/// Joint angles document as recorded by inspect
#[derive(Deserialize)]
struct JointsDoc {
    joints: Vec<f64>,
}

/// Base pose file written by set-base, read by move-to
const BASE_FILE: &str = ".jaka-cli-base.json";

/// Base pose document, xyz in mm
#[derive(Serialize, Deserialize)]
struct BaseDoc {
    x: f64,
    y: f64,
    z: f64,
}

#[tokio::main]
async fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_target(false)
        .init();

    let args = Cli::parse();

    // Print the static help text and exit
    if args.help {
        print!("{}", include_str!("help.txt"));
        return ExitCode::SUCCESS;
    }

    let Some(command) = &args.command else {
        error!(
            "No subcommand given. Expected one of status, power-on, power-off, estop-clear, inspect, inspect-pos, dh, set-base, rot, restore, move-to. Use --help for usage"
        );
        return ExitCode::FAILURE;
    };

    if args.dry_run {
        print_plan(&args, command);
        return ExitCode::SUCCESS;
    }

    match run(&args, command).await {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            error!("{e}");
            ExitCode::FAILURE
        }
    }
}

fn print_plan(args: &Cli, command: &Command) {
    match command {
        Command::Status => info!(
            "[dry-run] Will connect to controller {} and read state",
            args.ip
        ),
        Command::PowerOn => info!(
            "[dry-run] Will connect to controller {} and power on and enable",
            args.ip
        ),
        Command::PowerOff => info!(
            "[dry-run] Will connect to controller {} and disable servos and power off",
            args.ip
        ),
        Command::EstopClear => info!(
            "[dry-run] Will connect to controller {} and clear the e-stop error state",
            args.ip
        ),
        Command::Inspect => info!(
            "[dry-run] Will connect to controller {} and print the joint angles as JSON",
            args.ip
        ),
        Command::InspectPos => info!(
            "[dry-run] Will connect to controller {} and print the TCP position as JSON",
            args.ip
        ),
        Command::Dh => info!(
            "[dry-run] Will connect to controller {} and print the DH parameters as JSON",
            args.ip
        ),
        Command::Rot { joint, deg, speed } => {
            info!(
                "[dry-run] Will connect to controller {}, rotate joint J{} by {} degrees",
                args.ip, joint, deg
            );
            info!(
                "[dry-run] Speed {:.2} rad/s, about {:.1} seconds",
                speed,
                deg.to_radians().abs() / speed
            );
        }
        Command::Restore { file, speed } => {
            let name = file
                .as_deref()
                .map(|f| f.display().to_string())
                .unwrap_or_else(|| "built-in default pose".to_string());
            info!(
                "[dry-run] Will connect to controller {} and restore joints from {}",
                args.ip, name
            );
            info!("[dry-run] Speed {speed:.2} rad/s");
        }
        Command::MoveTo {
            x,
            y,
            z,
            speed,
            rx,
            ry,
            rz,
        } => {
            info!(
                "[dry-run] Will connect to controller {} and move the TCP to base + ({x}, {y}, {z}) mm",
                args.ip
            );
            let ori = match (rx, ry, rz) {
                (None, None, None) => "keep current".to_string(),
                _ => format!(
                    "rx={}, ry={}, rz={} degrees",
                    rx.unwrap_or(f64::NAN),
                    ry.unwrap_or(f64::NAN),
                    rz.unwrap_or(f64::NAN)
                ),
            };
            info!("[dry-run] Orientation: {ori}");
            info!("[dry-run] Linear speed {speed:.0} mm/s, out-of-workspace targets get clamped");
        }
        Command::SetBase => info!(
            "[dry-run] Will connect to controller {} and save the current TCP as the base pose",
            args.ip
        ),
    }
}

async fn run(args: &Cli, command: &Command) -> Result<(), String> {
    let ip = CString::new(args.ip.as_str()).map_err(|_| "IP contains invalid characters")?;

    // Keep SDK printf noise off stdout for the whole session. Machine-readable
    // output like the inspect JSON goes through the saved stdout descriptor
    let stdout_fd = binding::redirect_stdout_to_stderr();

    // Create the connection handle
    let mut handle: JKHD = 0;
    check("Connect controller", unsafe {
        binding::create_handler(ip.as_ptr(), &mut handle)
    })?;
    info!("Connected to controller {}", args.ip);

    // Always disconnect, even when the operation below fails
    let result = drive(&handle, command, stdout_fd).await;
    let ret = unsafe { binding::destory_handler(&handle) };
    if ret != binding::ERR_SUCC {
        error!("Disconnect failed with error code {ret}");
    }
    result
}

async fn drive(handle: &JKHD, command: &Command, stdout_fd: i32) -> Result<(), String> {
    // Read state first. status and estop-clear must work while the e-stop is pressed
    let mut st = RobotState::default();
    check("Read state", unsafe {
        binding::get_robot_state(handle, &mut st)
    })?;

    // Dispatch by subcommand. status and power-off never power on the robot
    match command {
        Command::Status => {
            print_state(&st);
            let mut cur = JointValue::zero();
            check("Read joint position", unsafe {
                binding::get_joint_position(handle, &mut cur)
            })?;
            info!("Current joint angles in degrees: {}", format_joints(&cur));
            Ok(())
        }
        Command::EstopClear => estop_clear(handle),
        Command::SetBase => set_base(handle),
        Command::Inspect => {
            let mut cur = JointValue::zero();
            check("Read joint position", unsafe {
                binding::get_joint_position(handle, &mut cur)
            })?;
            print_joints_json(stdout_fd, &cur);
            Ok(())
        }
        Command::InspectPos => {
            let mut cur = CartesianPose::zero();
            check("Read TCP position", unsafe {
                binding::get_tcp_position(handle, &mut cur)
            })?;
            print_head_pos_json(stdout_fd, &cur);
            Ok(())
        }
        Command::Dh => {
            let mut dh = DHParam::default();
            check("Read DH parameters", unsafe {
                binding::get_dh_param(handle, &mut dh)
            })?;
            info!("DH alpha in degrees: {}", fmt_array(&dh.alpha));
            info!("DH a in mm: {}", fmt_array(&dh.a));
            info!("DH d in mm: {}", fmt_array(&dh.d));
            info!(
                "DH joint_homeoff in degrees: {}",
                fmt_array(&dh.joint_homeoff)
            );
            print_dh_json(stdout_fd, &dh);
            Ok(())
        }
        Command::PowerOff => {
            if st.servo_enabled != 0 {
                check("Disable servos", unsafe { binding::disable_robot(handle) })?;
                info!("Servos disabled");
            }
            if st.powered_on != 0 {
                check("Power off", unsafe { binding::power_off(handle) })?;
                info!("Powered off");
            } else {
                info!("Already powered off");
            }
            Ok(())
        }
        Command::PowerOn
        | Command::Rot { .. }
        | Command::Restore { .. }
        | Command::MoveTo { .. } => {
            if st.estoped != 0 {
                return Err(
                    "E-stop is pressed. Release the button physically, then run estop-clear".into(),
                );
            }
            ensure_powered_enabled(handle, &st)?;
            match command {
                Command::Rot { joint, deg, speed } => rot(handle, *joint, *deg, *speed).await,
                Command::Restore { file, speed } => restore(handle, file.as_deref(), *speed).await,
                Command::MoveTo {
                    x,
                    y,
                    z,
                    speed,
                    rx,
                    ry,
                    rz,
                } => move_to(handle, *x, *y, *z, *speed, *rx, *ry, *rz).await,
                _ => Ok(()),
            }
        }
    }
}

/// Power on and enable the robot, skipping steps that are already done
fn ensure_powered_enabled(handle: &JKHD, st: &RobotState) -> Result<(), String> {
    if st.powered_on == 0 {
        check("Power on", unsafe { binding::power_on(handle) })?;
        info!("Powered on");
    } else {
        info!("Already powered on");
    }

    if st.servo_enabled == 0 {
        check("Enable robot", unsafe { binding::enable_robot(handle) })?;
        info!("Enabled");
    } else {
        info!("Already enabled");
    }
    Ok(())
}

/// Clear the e-stop error state. The physical button must be released first
fn estop_clear(handle: &JKHD) -> Result<(), String> {
    let mut in_estop: BOOL = 0;
    check("Check e-stop state", unsafe {
        binding::is_in_estop(handle, &mut in_estop)
    })?;

    if in_estop != 0 {
        warn!("E-stop is still active. Twist and pull the e-stop button to release it physically");
    }

    check("Clear error state", unsafe { binding::clear_error(handle) })?;
    info!("Error state cleared");

    let mut after: BOOL = 0;
    check("Re-check e-stop state", unsafe {
        binding::is_in_estop(handle, &mut after)
    })?;
    if after != 0 {
        Err(
            "E-stop is still active. Release the physical button first, then run estop-clear again"
                .into(),
        )
    } else {
        info!("E-stop released. Run power-on to bring the robot back");
        Ok(())
    }
}

/// Rotate one joint by deg degrees using an incremental move
async fn rot(handle: &JKHD, joint: i32, deg: f64, speed: f64) -> Result<(), String> {
    let speed = clamp_joint_speed(speed);

    // Read the current joint angles
    let mut cur = JointValue::zero();
    check("Read joint position", unsafe {
        binding::get_joint_position(handle, &mut cur)
    })?;
    info!("Current joint angles in degrees: {}", format_joints(&cur));

    if joint == 1 {
        warn!(
            "J1 is the base joint. A full rotation swings the whole arm. Make sure the area is clear"
        );
    }

    // Incremental move: only the target joint gets a delta, the rest stay zero
    let mut target = JointValue::zero();
    target.j_val[(joint - 1) as usize] = deg.to_radians();

    info!(
        "Rotating J{} by {} degrees at {:.2} rad/s",
        joint, deg, speed
    );

    let h = *handle;
    run_blocking_motion(handle, move || unsafe {
        binding::joint_move_extend(
            &h,
            &target,
            MoveMode::Incr,
            1, // is_block: block until the motion completes
            speed,
            1.0, // acc, rad/s^2
            0.0, // tol
            std::ptr::null::<OptionalCond>(),
        )
    })
    .await?;

    // Read the final joint angles
    let mut fin = JointValue::zero();
    check("Read final joint position", unsafe {
        binding::get_joint_position(handle, &mut fin)
    })?;
    info!("Final joint angles in degrees: {}", format_joints(&fin));
    info!("Done. Rotation complete");
    Ok(())
}

/// Restore the joint angles recorded by inspect from a JSON file. Without a
/// file the built-in default pose is used
async fn restore(handle: &JKHD, file: Option<&Path>, speed: f64) -> Result<(), String> {
    let speed = clamp_joint_speed(speed);

    // Load and parse the recorded joints, stored in degrees. The built-in
    // default pose is embedded at compile time so the binary stays standalone
    let text = match file {
        Some(path) => std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read {}: {e}", path.display()))?,
        None => include_str!("../stat-preset/default.json").to_string(),
    };
    let doc: JointsDoc = serde_json::from_str(&text).map_err(|e| format!("Invalid JSON: {e}"))?;
    if doc.joints.len() != 6 {
        return Err(format!(
            "joints must contain 6 values, got {}",
            doc.joints.len()
        ));
    }

    // Convert to radians as the absolute move target
    let mut target = JointValue::zero();
    for (i, v) in doc.joints.iter().enumerate() {
        target.j_val[i] = v.to_radians();
    }

    // Read the current joint angles
    let mut cur = JointValue::zero();
    check("Read joint position", unsafe {
        binding::get_joint_position(handle, &mut cur)
    })?;
    info!("Current joint angles in degrees: {}", format_joints(&cur));
    info!("Target joint angles in degrees: {}", format_joints(&target));

    info!(
        "Restoring joints from {} at {speed:.2} rad/s",
        file.map(|f| f.display().to_string())
            .unwrap_or_else(|| "built-in default pose".to_string())
    );
    let h = *handle;
    run_blocking_motion(handle, move || unsafe {
        binding::joint_move_extend(
            &h,
            &target,
            MoveMode::Abs,
            1, // is_block: block until the motion completes
            speed,
            1.0, // acc, rad/s^2
            0.0, // tol
            std::ptr::null::<OptionalCond>(),
        )
    })
    .await?;

    // Read the final joint angles
    let mut fin = JointValue::zero();
    check("Read final joint position", unsafe {
        binding::get_joint_position(handle, &mut fin)
    })?;
    info!("Final joint angles in degrees: {}", format_joints(&fin));
    info!("Restore complete");
    Ok(())
}

/// Run a blocking SDK motion call with Ctrl+C abort support
async fn run_blocking_motion(
    handle: &JKHD,
    motion: impl FnOnce() -> errno_t + Send + 'static,
) -> Result<(), String> {
    // The SDK move call blocks the calling thread. Run it on a blocking task so
    // the runtime stays responsive to the Ctrl+C signal
    let h = *handle;
    let mut task = tokio::task::spawn_blocking(motion);

    let outcome = tokio::select! {
        ret = &mut task => match ret {
            Ok(code) => check("Motion", code),
            Err(e) => Err(format!("Motion task panicked: {e}")),
        },
        _ = tokio::signal::ctrl_c() => {
            info!("Ctrl+C received, aborting motion");
            let h2 = h;
            let abort_ret = tokio::task::spawn_blocking(move || unsafe {
                binding::motion_abort(&h2)
            })
            .await;
            match abort_ret {
                Ok(code) => info!("Abort command returned code {code}"),
                Err(e) => warn!("Abort command failed: {e}"),
            }
            // Give the blocked move call a moment to unwind before disconnecting
            let _ = tokio::time::timeout(Duration::from_secs(5), &mut task).await;
            info!("Motion aborted by Ctrl+C");
            return Ok(());
        }
    };
    outcome
}

/// Clamp the joint speed to the controller's valid range
fn clamp_joint_speed(speed: f64) -> f64 {
    const MAX: f64 = 3.14; // rad/s, the fastest any JAKA joint can move
    const MIN: f64 = 0.01;
    if speed > MAX {
        warn!("Speed {speed:.2} rad/s exceeds the joint limit, clamped to {MAX:.2} rad/s");
        MAX
    } else if speed < MIN {
        warn!("Speed {speed:.2} rad/s is too slow, clamped to {MIN:.2} rad/s");
        MIN
    } else {
        speed
    }
}

/// Move the TCP to a fixed target: the base pose plus an xyz offset in mm.
/// Orientation is either the current one or the explicit rx/ry/rz in degrees.
/// The base pose is either the robot home position or the one saved by
/// set-base. Targets outside the workspace are clamped to the nearest
/// reachable point. Repeating the same command is a no-op once in position
async fn move_to(
    handle: &JKHD,
    x: f64,
    y: f64,
    z: f64,
    speed: f64,
    rx: Option<f64>,
    ry: Option<f64>,
    rz: Option<f64>,
) -> Result<(), String> {
    // The current TCP pose provides the orientation, which is kept unchanged
    let mut cur = CartesianPose::zero();
    check("Read TCP position", unsafe {
        binding::get_tcp_position(handle, &mut cur)
    })?;

    // Resolve the base pose: the saved one if present, otherwise home
    let (base, from_file) = load_base(handle)?;
    if from_file {
        info!(
            "Base pose from {} in mm: x={:.1}, y={:.1}, z={:.1}",
            BASE_FILE, base.tran.x, base.tran.y, base.tran.z
        );
    } else {
        info!(
            "Base pose is the robot home in mm: x={:.1}, y={:.1}, z={:.1}",
            base.tran.x, base.tran.y, base.tran.z
        );
    }

    // Target is the base pose plus the offset. Orientation comes from the
    // explicit angles when given, otherwise the current one is kept
    let mut target = base;
    target.tran.x += x;
    target.tran.y += y;
    target.tran.z += z;
    target.rpy.rx = rx.map(|v| v.to_radians()).unwrap_or(cur.rpy.rx);
    target.rpy.ry = ry.map(|v| v.to_radians()).unwrap_or(cur.rpy.ry);
    target.rpy.rz = rz.map(|v| v.to_radians()).unwrap_or(cur.rpy.rz);

    // Current joints are the IK reference to pick the nearest solution
    let mut ref_joint = JointValue::zero();
    check("Read joint position", unsafe {
        binding::get_joint_position(handle, &mut ref_joint)
    })?;

    let (goal, clamped) = resolve_with_clamp(handle, cur, target, &ref_joint)?;
    if clamped {
        // The clamped point is the current position, so the robot is already
        // at the workspace boundary and there is nothing to move
        let dist = ((goal.tran.x - cur.tran.x).powi(2)
            + (goal.tran.y - cur.tran.y).powi(2)
            + (goal.tran.z - cur.tran.z).powi(2))
        .sqrt();
        if dist < 1.0 {
            info!("Already at the workspace boundary, nothing to move");
            return Ok(());
        }
        warn!("Target is out of the workspace, clamped to the nearest reachable point");
    }
    info!(
        "Target TCP in mm: x={:.1}, y={:.1}, z={:.1}",
        goal.tran.x, goal.tran.y, goal.tran.z
    );

    info!("Moving linearly at {speed:.0} mm/s");
    let h = *handle;
    run_blocking_motion(handle, move || unsafe {
        binding::linear_move_extend(
            &h,
            &goal,
            MoveMode::Abs,
            1, // is_block: block until the motion completes
            speed,
            500.0, // acc, mm/s^2
            0.0,   // tol
            std::ptr::null::<OptionalCond>(),
        )
    })
    .await?;

    // Read the final TCP position
    let mut fin = CartesianPose::zero();
    check("Read final TCP position", unsafe {
        binding::get_tcp_position(handle, &mut fin)
    })?;
    info!(
        "Final TCP in mm: x={:.1}, y={:.1}, z={:.1}",
        fin.tran.x, fin.tran.y, fin.tran.z
    );
    info!("Done. Reached the target");
    Ok(())
}

/// Save the current TCP position as the base pose for move-to
fn set_base(handle: &JKHD) -> Result<(), String> {
    let mut cur = CartesianPose::zero();
    check("Read TCP position", unsafe {
        binding::get_tcp_position(handle, &mut cur)
    })?;

    let doc = BaseDoc {
        x: cur.tran.x,
        y: cur.tran.y,
        z: cur.tran.z,
    };
    let text = serde_json::to_string_pretty(&doc)
        .map_err(|e| format!("Serialize base pose failed: {e}"))?;
    let path = base_file_path();
    std::fs::write(&path, text).map_err(|e| format!("Failed to write {}: {e}", path.display()))?;
    info!(
        "Base pose saved to {} in mm: x={:.1}, y={:.1}, z={:.1}",
        path.display(),
        cur.tran.x,
        cur.tran.y,
        cur.tran.z
    );
    Ok(())
}

/// Load the base pose: the saved file if present, otherwise the robot home
/// position computed with forward kinematics at zero joints
fn load_base(handle: &JKHD) -> Result<(CartesianPose, bool), String> {
    let path = base_file_path();
    if let Ok(text) = std::fs::read_to_string(&path) {
        if let Ok(doc) = serde_json::from_str::<BaseDoc>(&text) {
            let mut base = CartesianPose::zero();
            base.tran.x = doc.x;
            base.tran.y = doc.y;
            base.tran.z = doc.z;
            return Ok((base, true));
        }
        warn!(
            "{} is invalid, falling back to the robot home",
            path.display()
        );
    }

    // Robot home: forward kinematics at zero joints
    let zero = JointValue::zero();
    let mut home = CartesianPose::zero();
    check("Compute home pose", unsafe {
        binding::kine_forward(handle, &zero, &mut home)
    })?;
    Ok((home, false))
}

fn base_file_path() -> std::path::PathBuf {
    std::env::var("HOME")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from("."))
        .join(BASE_FILE)
}

/// Inverse kinematics with clamping. When the target is unreachable, binary
/// search along the line from the current pose to the target for the nearest
/// reachable point. A candidate is only accepted when the whole straight path
/// from the current pose to it is reachable, so the later linear move cannot
/// fail midway
fn resolve_with_clamp(
    handle: &JKHD,
    cur: CartesianPose,
    target: CartesianPose,
    ref_joint: &JointValue,
) -> Result<(CartesianPose, bool), String> {
    if path_clear(handle, &cur, &target, ref_joint) {
        return Ok((target, false));
    }

    // lo is always reachable, hi is not
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    for _ in 0..20 {
        let mid = (lo + hi) / 2.0;
        let probe = lerp_pose(cur, target, mid);
        if path_clear(handle, &cur, &probe, ref_joint) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    if lo < 0.001 {
        // Nothing reachable beyond the current position, so the robot is
        // already at the workspace boundary. The caller detects this case
        return Ok((cur, true));
    }
    let goal = lerp_pose(cur, target, lo);
    Ok((goal, true))
}

/// Check that every sample along the straight path from from to to has a valid
/// inverse kinematics solution
fn path_clear(
    handle: &JKHD,
    from: &CartesianPose,
    to: &CartesianPose,
    ref_joint: &JointValue,
) -> bool {
    const SAMPLES: usize = 8;
    for i in 0..=SAMPLES {
        let t = i as f64 / SAMPLES as f64;
        let probe = lerp_pose(*from, *to, t);
        let mut joint = JointValue::zero();
        let ret = unsafe { binding::kine_inverse(handle, ref_joint, &probe, &mut joint) };
        if ret != binding::ERR_SUCC {
            return false;
        }
    }
    true
}

/// Linear interpolation between two poses, translation and orientation
fn lerp_pose(from: CartesianPose, to: CartesianPose, t: f64) -> CartesianPose {
    let lerp = |a: f64, b: f64| a + (b - a) * t;
    CartesianPose {
        tran: CartesianTran {
            x: lerp(from.tran.x, to.tran.x),
            y: lerp(from.tran.y, to.tran.y),
            z: lerp(from.tran.z, to.tran.z),
        },
        rpy: Rpy {
            rx: lerp(from.rpy.rx, to.rpy.rx),
            ry: lerp(from.rpy.ry, to.rpy.ry),
            rz: lerp(from.rpy.rz, to.rpy.rz),
        },
    }
}

fn print_state(st: &RobotState) {
    info!(
        "E-stop: {}",
        if st.estoped != 0 { "pressed" } else { "normal" }
    );
    info!("Powered: {}", if st.powered_on != 0 { "yes" } else { "no" });
    info!(
        "Enabled: {}",
        if st.servo_enabled != 0 { "yes" } else { "no" }
    );
}

fn format_joints(j: &JointValue) -> String {
    j.j_val
        .iter()
        .map(|v| format!("{:.1}", v.to_degrees()))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Print the TCP position relative to the base frame as a JSON object on the
/// original stdout. Positions are in mm, the base frame origin is [0,0,0]
fn print_head_pos_json(fd: i32, pose: &CartesianPose) {
    let json = serde_json::json!({
        "head_pos": [pose.tran.x, pose.tran.y, pose.tran.z],
        "base_pos": [0.0, 0.0, 0.0],
    });
    binding::write_to_fd(fd, &format!("{json}\n"));
}

/// Print the joint angles in degrees as a JSON object on the original stdout
fn print_joints_json(fd: i32, j: &JointValue) {
    let joints: Vec<f64> = j.j_val.iter().map(|v| v.to_degrees()).collect();
    let json = serde_json::json!({ "joints": joints }).to_string();
    binding::write_to_fd(fd, &format!("{json}\n"));
}

/// Print the DH parameters as a JSON object on the original stdout
fn print_dh_json(fd: i32, dh: &DHParam) {
    let json = serde_json::json!({
        "alpha": dh.alpha,
        "a": dh.a,
        "d": dh.d,
        "joint_homeoff": dh.joint_homeoff,
    });
    binding::write_to_fd(fd, &format!("{json}\n"));
}

fn fmt_array(vals: &[f64; 6]) -> String {
    vals.iter()
        .map(|v| format!("{:.3}", v.to_degrees()))
        .collect::<Vec<_>>()
        .join(", ")
}
