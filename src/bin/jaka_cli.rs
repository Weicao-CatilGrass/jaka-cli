//! JAKA robotic arm driver.
//!
//! Usage is documented in src/bin/jaka_cli_help.txt and printed by `jaka-cli --help`.
//! The log level is controlled by the RUST_LOG environment variable and defaults to info.
//!
//! The runtime is tokio. Blocking SDK calls run on blocking tasks so the async
//! runtime stays responsive to the Ctrl+C signal, which aborts any ongoing motion.

#[path = "../binding.rs"]
mod binding;
#[path = "../cli.rs"]
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
use tokio::io::{AsyncBufReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::task::JoinHandle;

/// TCP pose document as written by inspect-pos
#[derive(Deserialize)]
struct PoseDoc {
    head_pos: Vec<f64>,
    head_rpy: Option<Vec<f64>>,
}

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
        print!("{}", include_str!("jaka_cli_help.txt"));
        return ExitCode::SUCCESS;
    }

    let Some(command) = &args.command else {
        error!(
            "No subcommand given. Expected one of status, power-on, power-off, estop-clear, inspect, inspect-pos, dh, set-base, rot, restore, move-to, serve. Use --help for usage"
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
            rel,
            pose,
        } => {
            if let Some(path) = pose {
                info!(
                    "[dry-run] Will connect to controller {} and move the TCP to the pose from {}",
                    args.ip,
                    path.display()
                );
            } else {
                let (Some(x), Some(y), Some(z)) = (x, y, z) else {
                    error!("x, y, z are required unless --pose is given");
                    return;
                };
                let mode = if *rel { "base + offset" } else { "absolute" };
                info!(
                    "[dry-run] Will connect to controller {} and move the TCP to {mode} position ({x}, {y}, {z}) mm",
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
            }
            info!("[dry-run] Linear speed {speed:.0} mm/s, out-of-workspace targets get clamped");
        }
        Command::SetBase => info!(
            "[dry-run] Will connect to controller {} and save the current TCP as the base pose",
            args.ip
        ),
        Command::Serve { port, mock } => {
            if *mock {
                info!(
                    "[dry-run] Will serve the gamepad protocol on port {port} without a controller"
                );
            } else {
                info!(
                    "[dry-run] Will connect to controller {} and serve the gamepad protocol on port {port}",
                    args.ip
                );
            }
        }
    }
}

async fn run(args: &Cli, command: &Command) -> Result<(), String> {
    // The mock serve runs without a controller, only the protocol matters
    if let Command::Serve { port, mock } = command {
        if *mock {
            return serve(Backend::Mock(MockState::new()), *port, true).await;
        }
    }

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
    let result = match command {
        Command::Serve { port, .. } => serve(Backend::Real(handle), *port, false).await,
        _ => drive(&handle, command, stdout_fd).await,
    };
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
        Command::Serve { .. } => {
            // The serve subcommand is dispatched before drive, this arm is
            // unreachable but the match must stay exhaustive
            Err("serve is not a drive command".into())
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
                    rel,
                    pose,
                } => {
                    if pose.is_some() {
                        return move_to(
                            handle,
                            0.0,
                            0.0,
                            0.0,
                            *speed,
                            *rx,
                            *ry,
                            *rz,
                            *rel,
                            pose.as_deref(),
                        )
                        .await;
                    }
                    let (Some(x), Some(y), Some(z)) = (x, y, z) else {
                        return Err("x, y, z are required unless --pose is given".into());
                    };
                    move_to(handle, *x, *y, *z, *speed, *rx, *ry, *rz, *rel, None).await
                }
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
        None => include_str!("../../stat-preset/default.json").to_string(),
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

/// Move the TCP to an absolute base-frame position, or with rel=true to a
/// position relative to the base pose. Orientation is either the current one
/// or the explicit rx/ry/rz in degrees. Targets outside the workspace are
/// clamped to the nearest reachable point. Repeating the same command is a
/// no-op once in position
async fn move_to(
    handle: &JKHD,
    x: f64,
    y: f64,
    z: f64,
    speed: f64,
    rx: Option<f64>,
    ry: Option<f64>,
    rz: Option<f64>,
    rel: bool,
    pose: Option<&Path>,
) -> Result<(), String> {
    // The current TCP pose provides the orientation unless overridden
    let mut cur = CartesianPose::zero();
    check("Read TCP position", unsafe {
        binding::get_tcp_position(handle, &mut cur)
    })?;

    // A pose file from inspect-pos overrides position and orientation
    let (tx, ty, tz, trx, try_, trz) = if let Some(path) = pose {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read {}: {e}", path.display()))?;
        let doc: PoseDoc = serde_json::from_str(&text)
            .map_err(|e| format!("Invalid pose JSON in {}: {e}", path.display()))?;
        if doc.head_pos.len() != 3 {
            return Err(format!(
                "head_pos must have 3 values, got {}",
                doc.head_pos.len()
            ));
        }
        let rpy = doc.head_rpy.as_deref().unwrap_or(&[]);
        let (trx, try_, trz) = if rpy.len() == 3 {
            (Some(rpy[0]), Some(rpy[1]), Some(rpy[2]))
        } else {
            (None, None, None)
        };
        (
            doc.head_pos[0],
            doc.head_pos[1],
            doc.head_pos[2],
            trx,
            try_,
            trz,
        )
    } else {
        (x, y, z, rx, ry, rz)
    };

    let mut target = CartesianPose::zero();
    if rel {
        // Relative mode: the base pose plus the offset
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
        target.tran.x = base.tran.x + tx;
        target.tran.y = base.tran.y + ty;
        target.tran.z = base.tran.z + tz;
    } else {
        // Absolute mode: x y z are base-frame coordinates, the base pose is unused
        target.tran.x = tx;
        target.tran.y = ty;
        target.tran.z = tz;
    }

    // Orientation comes from the explicit angles when given, otherwise the
    // current one is kept
    target.rpy.rx = trx.map(|v| v.to_radians()).unwrap_or(cur.rpy.rx);
    target.rpy.ry = try_.map(|v| v.to_radians()).unwrap_or(cur.rpy.ry);
    target.rpy.rz = trz.map(|v| v.to_radians()).unwrap_or(cur.rpy.rz);

    // A large orientation change makes the interpolated straight path
    // unreachable almost always, so a joint move is used instead. The joint
    // path does not constrain the TCP, only the end pose must be reachable
    let big_ori_change = |a: f64, b: f64| (a - b).abs() > 30.0_f64.to_radians();
    let ori_jump = big_ori_change(cur.rpy.rx, target.rpy.rx)
        || big_ori_change(cur.rpy.ry, target.rpy.ry)
        || big_ori_change(cur.rpy.rz, target.rpy.rz);

    // Current joints are the IK reference to pick the nearest solution
    let mut ref_joint = JointValue::zero();
    check("Read joint position", unsafe {
        binding::get_joint_position(handle, &mut ref_joint)
    })?;

    info!(
        "Target TCP in mm: x={:.1}, y={:.1}, z={:.1}",
        target.tran.x, target.tran.y, target.tran.z
    );

    // A linear move needs the whole straight path clear and keeps the
    // orientation, it is the preferred fast path
    if ori_jump {
        info!("Large orientation change, using a joint move");
    } else if path_clear(handle, &cur, &target, &ref_joint) {
        info!("Moving linearly at {speed:.0} mm/s");
        let h = *handle;
        let linear = run_blocking_motion(handle, move || unsafe {
            binding::linear_move_extend(
                &h,
                &target,
                MoveMode::Abs,
                1, // is_block: block until the motion completes
                speed,
                500.0, // acc, mm/s^2
                0.0,   // tol
                std::ptr::null::<OptionalCond>(),
            )
        })
        .await;
        if let Err(e) = linear {
            warn!("Linear move failed: {e}, probing the reachable range instead");
        } else {
            return report_final(handle, &target).await;
        }
    } else {
        info!("Straight path is blocked, using a joint move");
    }

    // A joint move only needs the end pose reachable, and the reachability
    // is verified by executing the move. The SDK can return a mathematically
    // valid solution that the robot cannot execute, or abort the motion
    // midway, so the actual pose after every move decides. Try the target
    // itself first, then the mirrored orientation when none was given
    // explicitly. Every attempt may leave the robot somewhere along the way,
    // so the current pose is refreshed after each move. Targets below the
    // safe height are never attempted, the controller can lock up there
    if !too_low(&target) {
        if let Some(joint) = ik_solve(handle, &ref_joint, &cur, &target) {
            let actual = exec_and_read(handle, &joint).await?;
            if pose_close(&actual, &target) {
                return report_final(handle, &target).await;
            }
            cur = actual;
            ref_joint = read_joints(handle).await?;
        }
    } else {
        warn!("Target z is below the safe height, skipping the direct move");
    }
    let explicit_ori = pose.is_some() || rx.is_some() || ry.is_some() || rz.is_some();
    if !explicit_ori && !too_low(&target) {
        let mut mirrored = target;
        mirrored.rpy.rx = -target.rpy.rx;
        mirrored.rpy.ry = -target.rpy.ry;
        if let Some(joint) = ik_solve(handle, &ref_joint, &cur, &mirrored) {
            let actual = exec_and_read(handle, &joint).await?;
            if pose_close(&actual, &mirrored) {
                info!(
                    "Target orientation is not reachable, using the mirrored orientation rx={:.0}, ry={:.0}, rz={:.0}",
                    mirrored.rpy.rx.to_degrees(),
                    mirrored.rpy.ry.to_degrees(),
                    mirrored.rpy.rz.to_degrees()
                );
                return report_final(handle, &mirrored).await;
            }
            cur = actual;
            ref_joint = read_joints(handle).await?;
        }
    }

    // Nothing worked, so the target is unreachable. Walk back toward the
    // current position, probing the path with real joint moves. The robot
    // may arrive fully, stop midway or not move at all, so every landing
    // spot counts as a reachable point and the search continues from there.
    // The corridor ends at the safe height when the target is below it
    let mut safe_target = target;
    if too_low(&safe_target) {
        safe_target.tran.z = LOW_Z;
    }
    info!("Target is unreachable, probing the reachable point closest to it");
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut reached: Option<CartesianPose> = None;
    for _ in 0..20 {
        if hi - lo < 0.001 {
            break;
        }
        let mid = (lo + hi) / 2.0;
        let probe = lerp_pose(cur, safe_target, mid);
        if too_low(&probe) {
            hi = mid;
            continue;
        }
        log::debug!(
            "Probe t={mid:.4} at ({:.1}, {:.1}, {:.1})",
            probe.tran.x,
            probe.tran.y,
            probe.tran.z
        );
        let Some(joint) = ik_solve(handle, &ref_joint, &cur, &probe) else {
            log::debug!("  No IK solution");
            hi = mid;
            continue;
        };
        log::debug!("  IK solution: {}", format_joints(&joint));
        let actual = exec_and_read(handle, &joint).await?;
        let t = param_of(&cur, &safe_target, &actual);
        log::debug!(
            "  Landed at ({:.1}, {:.1}, {:.1}), t={t:.4}",
            actual.tran.x,
            actual.tran.y,
            actual.tran.z
        );
        if t > lo {
            lo = t;
            reached = Some(actual);
            ref_joint = read_joints(handle).await?;
        } else {
            hi = mid;
        }
    }
    let Some(goal) = reached else {
        info!("Already at the workspace boundary, nothing to move");
        return Ok(());
    };
    warn!("Target is out of the workspace, clamped to the nearest reachable point");
    info!(
        "Clamped TCP in mm: x={:.1}, y={:.1}, z={:.1}",
        goal.tran.x, goal.tran.y, goal.tran.z
    );
    report_final(handle, &goal).await
}

/// Execute a joint move and return the actual TCP pose after it. The SDK
/// can abort the motion midway or report an error after a completed motion,
/// so the actual position is read back instead of trusting the return code
async fn exec_and_read(handle: &JKHD, joint: &JointValue) -> Result<CartesianPose, String> {
    let h = *handle;
    let j = *joint;
    let ret = run_blocking_motion(handle, move || unsafe {
        binding::joint_move_extend(
            &h,
            &j,
            MoveMode::Abs,
            1,   // is_block: block until the motion completes
            3.0, // rad/s
            1.0, // acc, rad/s^2
            0.0, // tol
            std::ptr::null::<OptionalCond>(),
        )
    })
    .await;
    // The controller may have entered an error state that blocks further
    // moves, clear it right away so the probing can continue
    if ret.is_err() {
        log::debug!("Clearing the controller error state after a failed motion");
        let _ = unsafe { binding::clear_error(handle) };
    }
    // Read the actual pose, the robot may have moved fully, partially or
    // not at all regardless of the return code
    let mut actual = CartesianPose::zero();
    check("Read TCP position", unsafe {
        binding::get_tcp_position(handle, &mut actual)
    })?;
    if let Err(e) = ret {
        log::debug!("Motion returned an error but the robot may have moved: {e}");
    }
    Ok(actual)
}

/// Read the current joint angles
async fn read_joints(handle: &JKHD) -> Result<JointValue, String> {
    let mut cur = JointValue::zero();
    check("Read joint position", unsafe {
        binding::get_joint_position(handle, &mut cur)
    })?;
    Ok(cur)
}

/// The parameter of the projection of p onto the line from from to to
fn param_of(from: &CartesianPose, to: &CartesianPose, p: &CartesianPose) -> f64 {
    let dx = to.tran.x - from.tran.x;
    let dy = to.tran.y - from.tran.y;
    let dz = to.tran.z - from.tran.z;
    let len2 = dx * dx + dy * dy + dz * dz;
    if len2 < 1e-9 {
        return 0.0;
    }
    ((p.tran.x - from.tran.x) * dx + (p.tran.y - from.tran.y) * dy + (p.tran.z - from.tran.z) * dz)
        / len2
}

/// Read the final TCP position and report the accuracy against the goal
async fn report_final(handle: &JKHD, goal: &CartesianPose) -> Result<(), String> {
    let mut fin = CartesianPose::zero();
    check("Read final TCP position", unsafe {
        binding::get_tcp_position(handle, &mut fin)
    })?;
    let err = ((fin.tran.x - goal.tran.x).powi(2)
        + (fin.tran.y - goal.tran.y).powi(2)
        + (fin.tran.z - goal.tran.z).powi(2))
    .sqrt();
    info!(
        "Final TCP in mm: x={:.1}, y={:.1}, z={:.1}",
        fin.tran.x, fin.tran.y, fin.tran.z
    );
    info!("Position error: {err:.2} mm");
    if err > 2.0 {
        warn!("Large position error, check the robot");
    }
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

/// Commands of the gamepad control protocol, one command per line
enum ProtoCmd {
    Move(f64, f64, f64),
    Rotate(f64, f64, f64),
    Stop,
    EstopClear,
    PowerOn,
    PowerOff,
    Reset,
    Status,
}

/// The backend behind the serve protocol: a real controller or a simulated
/// state for testing the client side without the robot
#[derive(Clone, Copy)]
enum Backend {
    Real(JKHD),
    Mock(MockState),
}

/// Simulated robot state for serve --mock
#[derive(Clone, Copy)]
struct MockState {
    powered: bool,
    enabled: bool,
    estop: bool,
    /// TCP x y z in mm and rx ry rz in degrees, the home pose at startup
    head: [f64; 6],
    /// Joint angles in degrees, the default pose at startup
    joints: [f64; 6],
}

impl MockState {
    fn new() -> Self {
        Self {
            powered: false,
            enabled: false,
            estop: false,
            head: [-50.1, -6.0, 396.8, 0.0, 90.0, 0.0],
            joints: [0.0, 90.0, -90.0, 0.0, -90.0, 0.0],
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn status_json(&self) -> String {
        let json = serde_json::json!({
            "estop": self.estop,
            "powered": self.powered,
            "enabled": self.enabled,
            "joints": self.joints,
            "head_pos": [self.head[0], self.head[1], self.head[2]],
        });
        format!("status {json}\n")
    }
}

/// Run one protocol command against the simulated state, no robot involved
fn exec_mock(state: &mut MockState, cmd: ProtoCmd) -> Result<String, String> {
    match cmd {
        ProtoCmd::Move(dx, dy, dz) => {
            state.head[0] += dx;
            state.head[1] += dy;
            state.head[2] += dz;
            Ok("ok\n".into())
        }
        ProtoCmd::Rotate(drx, dry, drz) => {
            state.head[3] += drx;
            state.head[4] += dry;
            state.head[5] += drz;
            Ok("ok\n".into())
        }
        ProtoCmd::Stop => Ok("ok\n".into()),
        ProtoCmd::EstopClear => {
            state.estop = false;
            Ok("ok\n".into())
        }
        ProtoCmd::PowerOn => {
            state.powered = true;
            state.enabled = true;
            Ok("ok\n".into())
        }
        ProtoCmd::PowerOff => {
            state.enabled = false;
            state.powered = false;
            Ok("ok\n".into())
        }
        ProtoCmd::Reset => {
            state.reset();
            state.powered = true;
            state.enabled = true;
            Ok("ok\n".into())
        }
        ProtoCmd::Status => Ok(state.status_json()),
    }
}

/// Parse one protocol line: the command name followed by space separated numbers
fn parse_proto(line: &str) -> Result<ProtoCmd, String> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.is_empty() {
        return Err("empty line".into());
    }
    let num = |i: usize| -> Result<f64, String> {
        parts
            .get(i)
            .and_then(|s| s.parse().ok())
            .ok_or_else(|| format!("{} expects 3 numbers", parts[0]))
    };
    match parts[0] {
        "move" => Ok(ProtoCmd::Move(num(1)?, num(2)?, num(3)?)),
        "rotate" => Ok(ProtoCmd::Rotate(num(1)?, num(2)?, num(3)?)),
        "stop" => Ok(ProtoCmd::Stop),
        "estop-clear" => Ok(ProtoCmd::EstopClear),
        "poweron" => Ok(ProtoCmd::PowerOn),
        "poweroff" => Ok(ProtoCmd::PowerOff),
        "reset" => Ok(ProtoCmd::Reset),
        "status" => Ok(ProtoCmd::Status),
        other => Err(format!("unknown command: {other}")),
    }
}

/// Serve the gamepad control protocol over TCP. Incremental motion commands
/// abort the previous motion and start a new one, so the gamepad can stream
/// commands at its own frame rate
async fn serve(backend: Backend, port: u16, mock: bool) -> Result<(), String> {
    let listener = TcpListener::bind(("0.0.0.0", port))
        .await
        .map_err(|e| format!("Bind port {port} failed: {e}"))?;
    if mock {
        info!("Mock control server listening on port {port}, no controller involved");
    } else {
        info!("Control server listening on port {port}");
    }
    loop {
        let (stream, addr) = listener
            .accept()
            .await
            .map_err(|e| format!("Accept failed: {e}"))?;
        info!("Control client connected from {addr}");
        let mut backend = backend.clone();
        let mock = mock;
        tokio::spawn(async move {
            if let Err(e) = handle_client(&mut backend, stream, mock).await {
                warn!("Control client {addr} error: {e}");
            }
        });
    }
}

/// Serve one control client: read protocol lines, reply to each command
async fn handle_client(backend: &mut Backend, stream: TcpStream, mock: bool) -> Result<(), String> {
    let (reader, mut writer) = stream.into_split();
    let mut lines = tokio::io::BufReader::new(reader).lines();
    // The currently running incremental motion, aborted by the next command
    let mut motion: Option<JoinHandle<()>> = None;
    loop {
        let line = match lines.next_line().await {
            Ok(Some(l)) => l,
            Ok(None) => break,
            Err(e) => return Err(format!("Read line failed: {e}")),
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if mock {
            info!("[mock] {line}");
        }
        let reply = handle_proto(backend, &mut motion, line).await;
        writer
            .write_all(reply.as_bytes())
            .await
            .map_err(|e| format!("Write reply failed: {e}"))?;
    }
    // Disconnecting aborts any ongoing motion for safety
    if let Backend::Real(handle) = backend {
        abort_motion(handle, &mut motion).await;
    }
    info!("Control client disconnected");
    Ok(())
}

/// Execute one protocol command and return the reply line
async fn handle_proto(
    backend: &mut Backend,
    motion: &mut Option<JoinHandle<()>>,
    line: &str,
) -> String {
    let cmd = match parse_proto(line) {
        Ok(c) => c,
        Err(e) => return format!("err {e}\n"),
    };
    match exec_proto(backend, motion, cmd).await {
        Ok(reply) => reply,
        Err(e) => format!("err {e}\n"),
    }
}

/// Run one parsed protocol command against the backend
async fn exec_proto(
    backend: &mut Backend,
    motion: &mut Option<JoinHandle<()>>,
    cmd: ProtoCmd,
) -> Result<String, String> {
    match backend {
        Backend::Real(handle) => exec_real(handle, motion, cmd).await,
        Backend::Mock(state) => exec_mock(state, cmd),
    }
}

/// Run one parsed protocol command against the real controller
async fn exec_real(
    handle: &JKHD,
    motion: &mut Option<JoinHandle<()>>,
    cmd: ProtoCmd,
) -> Result<String, String> {
    match cmd {
        ProtoCmd::Move(dx, dy, dz) => {
            start_linear(handle, motion, dx, dy, dz, 0.0, 0.0, 0.0).await;
            Ok("ok\n".into())
        }
        ProtoCmd::Rotate(drx, dry, drz) => {
            start_linear(handle, motion, 0.0, 0.0, 0.0, drx, dry, drz).await;
            Ok("ok\n".into())
        }
        ProtoCmd::Stop => {
            abort_motion(handle, motion).await;
            Ok("ok\n".into())
        }
        ProtoCmd::EstopClear => {
            abort_motion(handle, motion).await;
            estop_clear(handle)?;
            Ok("ok\n".into())
        }
        ProtoCmd::PowerOn => {
            abort_motion(handle, motion).await;
            let mut st = RobotState::default();
            check("Read state", unsafe {
                binding::get_robot_state(handle, &mut st)
            })?;
            if st.estoped != 0 {
                return Err("E-stop is pressed, release the button first".into());
            }
            ensure_powered_enabled(handle, &st)?;
            Ok("ok\n".into())
        }
        ProtoCmd::PowerOff => {
            abort_motion(handle, motion).await;
            let mut st = RobotState::default();
            check("Read state", unsafe {
                binding::get_robot_state(handle, &mut st)
            })?;
            if st.servo_enabled != 0 {
                check("Disable servos", unsafe { binding::disable_robot(handle) })?;
            }
            if st.powered_on != 0 {
                check("Power off", unsafe { binding::power_off(handle) })?;
            }
            Ok("ok\n".into())
        }
        ProtoCmd::Reset => {
            abort_motion(handle, motion).await;
            let mut st = RobotState::default();
            check("Read state", unsafe {
                binding::get_robot_state(handle, &mut st)
            })?;
            if st.estoped != 0 {
                return Err("E-stop is pressed, release the button first".into());
            }
            ensure_powered_enabled(handle, &st)?;
            restore(handle, None, 3.14).await?;
            Ok("ok\n".into())
        }
        ProtoCmd::Status => {
            let mut st = RobotState::default();
            check("Read state", unsafe {
                binding::get_robot_state(handle, &mut st)
            })?;
            let mut joints = JointValue::zero();
            check("Read joint position", unsafe {
                binding::get_joint_position(handle, &mut joints)
            })?;
            let mut tcp = CartesianPose::zero();
            check("Read TCP position", unsafe {
                binding::get_tcp_position(handle, &mut tcp)
            })?;
            let j: Vec<f64> = joints.j_val.iter().map(|v| v.to_degrees()).collect();
            let json = serde_json::json!({
                "estop": st.estoped != 0,
                "powered": st.powered_on != 0,
                "enabled": st.servo_enabled != 0,
                "joints": j,
                "head_pos": [tcp.tran.x, tcp.tran.y, tcp.tran.z],
            });
            Ok(format!("status {json}\n"))
        }
    }
}

/// Abort the SDK motion and wait for the background task to finish
async fn abort_motion(handle: &JKHD, motion: &mut Option<JoinHandle<()>>) {
    if let Some(task) = motion.take() {
        let _ = unsafe { binding::motion_abort(handle) };
        let _ = tokio::time::timeout(Duration::from_secs(5), task).await;
    }
}

/// Abort the current motion and start a new incremental linear motion in the
/// background. The reply returns immediately so the gamepad can stream the
/// next command
async fn start_linear(
    handle: &JKHD,
    motion: &mut Option<JoinHandle<()>>,
    dx: f64,
    dy: f64,
    dz: f64,
    drx: f64,
    dry: f64,
    drz: f64,
) {
    abort_motion(handle, motion).await;
    let h = *handle;
    let mut target = CartesianPose::zero();
    target.tran.x = dx;
    target.tran.y = dy;
    target.tran.z = dz;
    target.rpy.rx = drx.to_radians();
    target.rpy.ry = dry.to_radians();
    target.rpy.rz = drz.to_radians();
    *motion = Some(tokio::spawn(async move {
        let ret = tokio::task::spawn_blocking(move || unsafe {
            binding::linear_move_extend(
                &h,
                &target,
                MoveMode::Incr,
                1,     // is_block: block until the motion completes
                200.0, // mm/s, gentle for gamepad control
                500.0, // acc, mm/s^2
                0.0,   // tol
                std::ptr::null::<OptionalCond>(),
            )
        })
        .await;
        match ret {
            Ok(code) if code == binding::ERR_SUCC => {}
            Ok(code) => warn!("Incremental motion failed: {}", binding::err_name(code)),
            Err(e) => warn!("Incremental motion task failed: {e}"),
        }
    }));
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

/// Check an IK solution against the JAKA Mini joint travel limits in degrees
fn joint_ok(j: &JointValue) -> bool {
    const LIMITS: [(f64, f64); 6] = [
        (-360.0, 360.0), // J1
        (-120.0, 120.0), // J2, JAKA Mini
        (-360.0, 360.0), // J3
        (-360.0, 360.0), // J4
        (-120.0, 120.0), // J5, JAKA Mini
        (-360.0, 360.0), // J6
    ];
    j.j_val.iter().zip(LIMITS.iter()).all(|(v, (lo, hi))| {
        let deg = v.to_degrees();
        deg >= *lo && deg <= *hi
    })
}

/// Run the SDK IK solver from several reference joints and return the first
/// solution that matches the target pose. The solver is reference-dependent
/// and can return out-of-limit or pose-compromised solutions, so every
/// solution is verified with forward kinematics and the joint limits
fn ik_solve(
    handle: &JKHD,
    current: &JointValue,
    cur: &CartesianPose,
    pose: &CartesianPose,
) -> Option<JointValue> {
    let refs = vec![
        *current,
        JointValue::zero(),
        home_joints(),
        aimed_reference(current, cur, pose),
    ];
    let mut joint = JointValue::zero();
    for r in refs {
        let ret = unsafe { binding::kine_inverse(handle, &r, pose, &mut joint) };
        if ret != binding::ERR_SUCC || !joint_ok(&joint) {
            continue;
        }
        // Verify the solution really produces the target pose. The solver
        // can return a solution with a compromised orientation that the
        // controller refuses to execute
        let mut fk = CartesianPose::zero();
        let fret = unsafe { binding::kine_forward(handle, &joint, &mut fk) };
        if fret == binding::ERR_SUCC && pose_close(&fk, pose) {
            return Some(joint);
        }
    }
    None
}

/// Compare two poses within the FK tolerance, positions in mm and angles in
/// radians
fn pose_close(a: &CartesianPose, b: &CartesianPose) -> bool {
    const POS_TOL: f64 = 1.0; // mm
    const ORI_TOL: f64 = 5.0; // degrees
    let pos = ((a.tran.x - b.tran.x).powi(2)
        + (a.tran.y - b.tran.y).powi(2)
        + (a.tran.z - b.tran.z).powi(2))
    .sqrt();
    pos < POS_TOL
        && ang_diff(a.rpy.rx, b.rpy.rx).to_degrees().abs() < ORI_TOL
        && ang_diff(a.rpy.ry, b.rpy.ry).to_degrees().abs() < ORI_TOL
        && ang_diff(a.rpy.rz, b.rpy.rz).to_degrees().abs() < ORI_TOL
}

/// Angular difference wrapped into [-pi, pi]
fn ang_diff(a: f64, b: f64) -> f64 {
    let d = (a - b) % (2.0 * std::f64::consts::PI);
    (d + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}

/// Build an IK reference by rotating the arm plane of the current joints
/// toward the target. J1 is the only joint that rotates the arm plane, so
/// the polar angle delta is applied to J1 alone while the elbow joints keep
/// their current values, which stay close to the solution
fn aimed_reference(
    current: &JointValue,
    cur: &CartesianPose,
    target: &CartesianPose,
) -> JointValue {
    let mut ref_j = *current;
    let cur_polar = cur.tran.y.atan2(cur.tran.x);
    let target_polar = target.tran.y.atan2(target.tran.x);
    ref_j.j_val[0] += target_polar - cur_polar;
    ref_j
}

/// The lowest TCP height in mm the move-to probing is allowed to explore.
/// Below this the controller can abort motions and lock up until the error
/// state is cleared, so the auto-probing never goes there
const LOW_Z: f64 = 100.0;

/// Check whether a pose is below the safe probing height
fn too_low(p: &CartesianPose) -> bool {
    p.tran.z < LOW_Z
}

/// The default home pose in radians, used as an IK reference
fn home_joints() -> JointValue {
    let mut j = JointValue::zero();
    j.j_val[1] = std::f64::consts::FRAC_PI_2; // J2: 90 deg
    j.j_val[2] = -std::f64::consts::FRAC_PI_2; // J3: -90 deg
    j.j_val[4] = -std::f64::consts::FRAC_PI_2; // J5: -90 deg
    j
}

/// Check that every sample along the straight line has an IK solution and
/// stays above the safe height, a linear move passes through all of them
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
        if too_low(&probe) || ik_solve(handle, ref_joint, from, &probe).is_none() {
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

/// Print the TCP pose relative to the base frame as a JSON object on the
/// original stdout. Positions are in mm, orientations in degrees
fn print_head_pos_json(fd: i32, pose: &CartesianPose) {
    let json = serde_json::json!({
        "head_pos": [pose.tran.x, pose.tran.y, pose.tran.z],
        "head_rpy": [
            pose.rpy.rx.to_degrees(),
            pose.rpy.ry.to_degrees(),
            pose.rpy.rz.to_degrees(),
        ],
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

#[cfg(test)]
mod tests {
    use super::MockState;
    use super::ProtoCmd;
    use super::exec_mock;
    use super::parse_proto;

    #[test]
    fn parse_motion_commands() {
        match parse_proto("move 10 -5 2.5").unwrap() {
            ProtoCmd::Move(x, y, z) => {
                assert_eq!((x, y, z), (10.0, -5.0, 2.5));
            }
            _ => panic!("expected move"),
        }
        match parse_proto("rotate 0 0 -15").unwrap() {
            ProtoCmd::Rotate(x, y, z) => {
                assert_eq!((x, y, z), (0.0, 0.0, -15.0));
            }
            _ => panic!("expected rotate"),
        }
    }

    #[test]
    fn parse_simple_commands() {
        assert!(matches!(parse_proto("stop").unwrap(), ProtoCmd::Stop));
        assert!(matches!(
            parse_proto("estop-clear").unwrap(),
            ProtoCmd::EstopClear
        ));
        assert!(matches!(parse_proto("poweron").unwrap(), ProtoCmd::PowerOn));
        assert!(matches!(
            parse_proto("poweroff").unwrap(),
            ProtoCmd::PowerOff
        ));
        assert!(matches!(parse_proto("reset").unwrap(), ProtoCmd::Reset));
        assert!(matches!(parse_proto("status").unwrap(), ProtoCmd::Status));
    }

    #[test]
    fn parse_rejects_bad_lines() {
        assert!(parse_proto("").is_err());
        assert!(parse_proto("move 1 2").is_err());
        assert!(parse_proto("move a b c").is_err());
        assert!(parse_proto("fly").is_err());
    }

    #[test]
    fn mock_state_moves_and_reports() {
        let mut st = MockState::new();
        assert!(exec_mock(&mut st, ProtoCmd::PowerOn).is_ok());
        assert!(st.powered && st.enabled);
        assert!(exec_mock(&mut st, ProtoCmd::Move(10.0, -5.0, 0.0)).is_ok());
        assert_eq!(st.head[0], -40.1);
        assert_eq!(st.head[1], -11.0);
        assert!(exec_mock(&mut st, ProtoCmd::Rotate(0.0, 0.0, 15.0)).is_ok());
        assert_eq!(st.head[5], 15.0);
        let reply = exec_mock(&mut st, ProtoCmd::Status).unwrap();
        assert!(reply.contains("\"powered\":true"));
        assert!(reply.contains("-40.1"));
        assert!(exec_mock(&mut st, ProtoCmd::Reset).is_ok());
        assert_eq!(st.head[0], -50.1);
    }
}
