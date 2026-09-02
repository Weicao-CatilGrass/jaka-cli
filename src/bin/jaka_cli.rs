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
use std::sync::Arc;
use std::time::{Duration, Instant};

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
use tokio::sync::Mutex;
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
            json,
        } => {
            if let Some(path) = pose {
                info!(
                    "[dry-run] Will connect to controller {} and move the TCP to the pose from {}",
                    args.ip,
                    path.display()
                );
            } else if json.is_some() {
                info!(
                    "[dry-run] Will connect to controller {} and move the TCP to the pose from --json",
                    args.ip
                );
            } else {
                let (Some(x), Some(y), Some(z)) = (x, y, z) else {
                    error!("x, y, z are required unless --pose or --json is given");
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
    // The mock serve runs without a controller, the simulated robot follows
    // the same state machine and jog model as the real one
    if let Command::Serve { port, mock } = command {
        if *mock {
            return serve(Backend::Mock(MockState::new()), *port).await;
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
        Command::Serve { port, .. } => {
            serve(
                Backend::Real {
                    handle,
                    servo: ServoState::new(),
                },
                *port,
            )
            .await
        }
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
                    json,
                } => {
                    if pose.is_some() || json.is_some() {
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
                            json.as_deref(),
                        )
                        .await;
                    }
                    let (Some(x), Some(y), Some(z)) = (x, y, z) else {
                        return Err("x, y, z are required unless --pose or --json is given".into());
                    };
                    move_to(handle, *x, *y, *z, *speed, *rx, *ry, *rz, *rel, None, None).await
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
    json: Option<&str>,
) -> Result<(), String> {
    // The current TCP pose provides the orientation unless overridden
    let mut cur = CartesianPose::zero();
    check("Read TCP position", unsafe {
        binding::get_tcp_position(handle, &mut cur)
    })?;

    // A pose document from a file or --json overrides position and
    // orientation, both are inspect-pos JSON
    let doc_text: Option<(&str, String)> = match (pose, json) {
        (Some(path), _) => Some((
            "pose file",
            std::fs::read_to_string(path)
                .map_err(|e| format!("Failed to read {}: {e}", path.display()))?,
        )),
        (None, Some(text)) => Some(("--json", text.to_string())),
        (None, None) => None,
    };
    let (tx, ty, tz, trx, try_, trz) = if let Some((src, text)) = doc_text {
        let doc: PoseDoc =
            serde_json::from_str(&text).map_err(|e| format!("Invalid pose JSON in {src}: {e}"))?;
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
    /// Continuous velocity, translations in mm/s, rotations in deg/s
    Vel(f64, f64, f64, f64, f64, f64),
    EstopClear,
    PowerOn,
    PowerOff,
    Reset,
    Status,
}

/// Safe jog velocity limits, translations in mm/s and rotations in deg/s
const MAX_LIN_VEL: f64 = 100.0;
const MAX_ROT_VEL: f64 = 20.0;

/// Clamp the requested velocities to the safe limits
fn clamp_vel(v: [f64; 6]) -> [f64; 6] {
    let mut out = [0.0; 6];
    for i in 0..6 {
        let limit = if i < 3 { MAX_LIN_VEL } else { MAX_ROT_VEL };
        out[i] = v[i].clamp(-limit, limit);
    }
    out
}

/// The backend behind the serve protocol: a real controller or a simulated
/// robot that follows the same state machine, so the mock behaves like the
/// real thing. The protocol logic in exec_cmd is shared by both
#[derive(Clone, Copy)]
enum Backend {
    Real {
        handle: JKHD,
        /// The servo pulse state of the real arm
        servo: ServoState,
    },
    Mock(MockState),
}

/// The real-arm servo state. Velocity commands only set the target, a
/// periodic pulse loop ramps the fed velocity toward it and streams absolute
/// poses to the controller, so every axis moves at the same time
#[derive(Clone, Copy)]
struct ServoState {
    /// Whether servo mode is active on the controller
    active: bool,
    /// The velocity fed to the controller in the last pulse, in mm/s and deg/s
    vel: [f64; 6],
    /// The target velocity of the protocol, in mm/s and deg/s
    target: [f64; 6],
    /// Absolute TCP translation of the model, mm
    tran: [f64; 3],
    /// Absolute orientation of the model, a row-major 3x3 rotation matrix.
    /// The matrix accumulates base-axis rotations, the rpy sent to the SDK
    /// is extracted from it, so the model stays valid through the gimbal
    /// lock of the rpy representation
    rot: [f64; 9],
    /// How many pulses were fed since the last calibration readback
    pulses: u32,
}

impl ServoState {
    fn new() -> Self {
        Self {
            active: false,
            vel: [0.0; 6],
            target: [0.0; 6],
            tran: [0.0; 3],
            rot: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            pulses: 0,
        }
    }
}

/// The servo pulse period. The controller interpolates each command over
/// 8 ms, so the pulse rate must stay above one command per cycle
const SERVO_PERIOD: Duration = Duration::from_millis(8);
/// The longest wall-clock time one pulse may integrate, guards against a
/// paused runtime sending a huge delta after a stall
const MAX_PULSE_DT: f64 = 0.05;
/// One pulse is fed in blocks of at most this many seconds, so a delayed
/// tick never delivers a single delta larger than one cycle can interpolate
const PULSE_STEP: f64 = 0.01;
/// How long a stopped arm keeps the servo stream alive before leaving servo
/// mode, so a resting client does not occupy the controller forever
const IDLE_END: f64 = 0.25;
/// The acceleration of the velocity ramp, translations in mm/s^2 and
/// rotations in deg/s^2. Sudden stick changes become smooth speed changes
const RAMP_LIN: f64 = 800.0;
const RAMP_ROT: f64 = 120.0;
/// Joint filter limits of the servo NLF, in deg/s, deg/s^2 and deg/s^3.
/// The controller smooths the joint trajectory below these caps, so a
/// rotating move never demands more than the joints can deliver
const NLF_VR: f64 = 150.0;
const NLF_AR: f64 = 500.0;
const NLF_JR: f64 = 2000.0;
/// Read the real TCP pose back this often and resync the model when the arm
/// stopped or drifted, every N pulses
const CALIB_EVERY: u32 = 60;
/// A model error above this distance or angle forces a hard resync
const CALIB_MAX_ERR: f64 = 10.0; // mm
const CALIB_MAX_ORI: f64 = 0.2; // radians

/// A readable name for the servo error codes that abort the motion, from
/// the SDK error code table. Joint codes carry the joint number in the
/// high byte
fn servo_err_hint(code: errno_t) -> String {
    let c = code.unsigned_abs();
    let low = c & 0xFFFF;
    let joint = (c >> 16) as usize;
    match low {
        0x0030 if joint < 6 => format!("joint {} speed over limit", joint + 1),
        0x8480 if joint < 6 => format!("joint {} forward tracking error", joint + 1),
        0x8481 if joint < 6 => format!("joint {} reverse tracking error", joint + 1),
        _ => match c {
            0x0F0001 => "robot is powered off".into(),
            0x0F0002 => "robot is disabled".into(),
            0x0F0003 => "operation not allowed in this mode".into(),
            0x0F0004 | 0x0F2051 => "inverse kinematics failed".into(),
            0x0F0010 => "invalid command speed".into(),
            0x0F0074 => "TCP speed limit reached".into(),
            0x0F0078 => "servo enable too frequent".into(),
            _ => format!("error code {code}"),
        },
    }
}

/// Rotation matrix about X, Y and Z of the base frame, row-major
fn rot_x(a: f64) -> [f64; 9] {
    let (c, s) = (a.cos(), a.sin());
    [1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c]
}

fn rot_y(a: f64) -> [f64; 9] {
    let (c, s) = (a.cos(), a.sin());
    [c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c]
}

fn rot_z(a: f64) -> [f64; 9] {
    let (c, s) = (a.cos(), a.sin());
    [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
}

/// Matrix product a times b, both row-major 3x3
fn mat_mul(a: [f64; 9], b: [f64; 9]) -> [f64; 9] {
    let mut out = [0.0; 9];
    for r in 0..3 {
        for c in 0..3 {
            out[r * 3 + c] = (0..3).map(|k| a[r * 3 + k] * b[k * 3 + c]).sum();
        }
    }
    out
}

/// Rpy to matrix with the SDK convention R = Rz(rz) Ry(ry) Rx(rx)
fn rpy_to_rot(rx: f64, ry: f64, rz: f64) -> [f64; 9] {
    mat_mul(rot_z(rz), mat_mul(rot_y(ry), rot_x(rx)))
}

/// Matrix to rpy with the SDK convention R = Rz(rz) Ry(ry) Rx(rx). The
/// representation locks when ry approaches +-90 degrees, the locked angle
/// is then folded into rx with rz zeroed, matching the controller output
fn rot_to_rpy(r: [f64; 9]) -> (f64, f64, f64) {
    // R20 = -sin(ry), R21 = cos(ry) sin(rx), R22 = cos(ry) cos(rx)
    let sy = (-r[6]).clamp(-1.0, 1.0);
    let ry = sy.asin();
    let cy = ry.cos();
    if cy > 1e-9 {
        let rx = r[7].atan2(r[8]);
        // R10 = sin(rz) cos(ry), R00 = cos(rz) cos(ry)
        let rz = r[3].atan2(r[0]);
        (rx, ry, rz)
    } else if ry > 0.0 {
        // Lock at ry = +90: R01 = sin(rx - rz), R02 = cos(rx - rz)
        let rx = r[1].atan2(r[2]);
        (rx, ry, 0.0)
    } else {
        // Lock at ry = -90: R01 = -sin(rx + rz), R02 = -cos(rx + rz)
        let rx = (-r[1]).atan2(-r[2]);
        (rx, ry, 0.0)
    }
}

/// The angle of the rotation between two matrices, radians
fn rot_angle(a: [f64; 9], b: [f64; 9]) -> f64 {
    // Trace of a * b^T is the element-wise product of a and b
    let mut t = 0.0;
    for i in 0..9 {
        t += a[i] * b[i];
    }
    ((t - 1.0) / 2.0).clamp(-1.0, 1.0).acos()
}

/// Read the controller error state, None while the controller is normal
fn read_robot_error(handle: &JKHD) -> Option<(errno_t, String)> {
    let mut st = binding::RobotStatusSimple::default();
    if unsafe { binding::get_robot_status_simple(handle, &mut st) } != binding::ERR_SUCC {
        return None;
    }
    if st.errcode == 0 {
        return None;
    }
    let bytes: Vec<u8> = st
        .errmsg
        .iter()
        .map(|&c| c as u8)
        .take_while(|&b| b != 0)
        .collect();
    let msg = String::from_utf8_lossy(&bytes).trim().to_string();
    Some((st.errcode, msg))
}

impl Backend {
    fn is_mock(&self) -> bool {
        matches!(self, Backend::Mock(_))
    }

    /// Whether the e-stop is active. The mock advances its simulation clock
    /// first, so the report reflects the motion since the last command
    fn in_estop(&mut self) -> Result<bool, String> {
        match self {
            Backend::Real { handle, .. } => {
                let mut st = RobotState::default();
                check("Read state", unsafe {
                    binding::get_robot_state(handle, &mut st)
                })?;
                Ok(st.estoped != 0)
            }
            Backend::Mock(state) => {
                state.advance();
                Ok(state.estop)
            }
        }
    }

    /// Enter servo mode on the real arm before the first motion command. On
    /// the mock this checks that the simulated arm can move at all
    fn ensure_servo(&mut self) -> Result<(), String> {
        match self {
            Backend::Real { handle, servo } => {
                if !servo.active {
                    // Set the joint filter before entering servo mode, the
                    // controller rejects filter changes while servoing
                    if unsafe { binding::servo_move_use_joint_NLF(handle, NLF_VR, NLF_AR, NLF_JR) }
                        != binding::ERR_SUCC
                    {
                        warn!("Joint NLF filter is not supported, running without it");
                    }
                    check("Enable servo mode", unsafe {
                        binding::servo_move_enable(handle, 1)
                    })?;
                    // Anchor the absolute pose model on the real pose
                    let mut tcp = CartesianPose::zero();
                    check("Read TCP pose", unsafe {
                        binding::get_tcp_position(handle, &mut tcp)
                    })?;
                    servo.active = true;
                    servo.vel = [0.0; 6];
                    servo.tran = [tcp.tran.x, tcp.tran.y, tcp.tran.z];
                    servo.rot = rpy_to_rot(tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz);
                    servo.pulses = 0;
                    info!("Servo mode enabled");
                }
                Ok(())
            }
            Backend::Mock(state) => state.check_can_move(),
        }
    }

    /// Set the protocol target velocity, translations in mm/s and rotations
    /// in deg/s. The pulse loop feeds it to the arm gradually
    fn set_target(&mut self, target: [f64; 6]) {
        match self {
            Backend::Real { servo, .. } => servo.target = target,
            Backend::Mock(state) => {
                // Integrate the old jog up to now before replacing it
                state.advance();
                state.jog = target;
            }
        }
    }

    /// The protocol target velocity
    fn target(&self) -> [f64; 6] {
        match self {
            Backend::Real { servo, .. } => servo.target,
            Backend::Mock(state) => state.jog,
        }
    }

    /// Feed one servo pulse: ramp the fed velocity toward the target and
    /// advance the absolute pose model, which is sent as the next command.
    /// The orientation accumulates as a matrix, so the model stays valid
    /// through the gimbal lock of the rpy representation. The mock
    /// integrates the jog on wall-clock time instead, the same motion model
    fn pulse(&mut self, dt: f64) -> Result<(), String> {
        match self {
            Backend::Real { handle, servo } => {
                if !servo.active {
                    return Ok(());
                }
                let mut left = dt.min(MAX_PULSE_DT);
                while left > 0.0 {
                    // One block per interpolation cycle, a stalled runtime
                    // must not deliver one huge delta in a single command
                    let step = left.min(PULSE_STEP);
                    let mut d = [0.0; 6];
                    for i in 0..6 {
                        let limit = if i < 3 { RAMP_LIN } else { RAMP_ROT };
                        let ramp = limit * step;
                        let cur = servo.vel[i];
                        let tgt = servo.target[i];
                        servo.vel[i] = if tgt > cur {
                            (cur + ramp).min(tgt)
                        } else {
                            (cur - ramp).max(tgt)
                        };
                        d[i] = servo.vel[i] * step;
                    }
                    // Advance the translation and rotate about the base axes
                    for i in 0..3 {
                        servo.tran[i] += d[i];
                    }
                    if d[3] != 0.0 {
                        servo.rot = mat_mul(rot_x(d[3].to_radians()), servo.rot);
                    }
                    if d[4] != 0.0 {
                        servo.rot = mat_mul(rot_y(d[4].to_radians()), servo.rot);
                    }
                    if d[5] != 0.0 {
                        servo.rot = mat_mul(rot_z(d[5].to_radians()), servo.rot);
                    }
                    // Send the absolute pose, the rpy is extracted from the
                    // matrix right before the call
                    let mut pose = CartesianPose::zero();
                    pose.tran.x = servo.tran[0];
                    pose.tran.y = servo.tran[1];
                    pose.tran.z = servo.tran[2];
                    let (rx, ry, rz) = rot_to_rpy(servo.rot);
                    pose.rpy.rx = rx;
                    pose.rpy.ry = ry;
                    pose.rpy.rz = rz;
                    let ret = unsafe { binding::servo_p(handle, &pose, MoveMode::Abs, 1) };
                    if ret != binding::ERR_SUCC {
                        return Err(format!("Servo pulse rejected: {}", servo_err_hint(ret)));
                    }
                    left -= step;
                }
                // Read the real pose back now and then, so a stopped arm
                // never gets chased by a model that kept advancing
                servo.pulses += 1;
                if servo.pulses % CALIB_EVERY == 0 {
                    let mut real = CartesianPose::zero();
                    let ret = unsafe { binding::get_tcp_position(handle, &mut real) };
                    if ret != binding::ERR_SUCC {
                        return Err(format!(
                            "Servo calibration read failed: {}",
                            servo_err_hint(ret)
                        ));
                    }
                    let dx = real.tran.x - servo.tran[0];
                    let dy = real.tran.y - servo.tran[1];
                    let dz = real.tran.z - servo.tran[2];
                    let err = (dx * dx + dy * dy + dz * dz).sqrt();
                    let real_rot = rpy_to_rot(real.rpy.rx, real.rpy.ry, real.rpy.rz);
                    let ori = rot_angle(servo.rot, real_rot);
                    if err > CALIB_MAX_ERR || ori > CALIB_MAX_ORI {
                        warn!(
                            "Servo model drifted {err:.1} mm / {:.1} deg from the real pose",
                            ori.to_degrees()
                        );
                        // Report the controller error behind the stopped arm
                        if let Some((code, msg)) = read_robot_error(handle) {
                            warn!(
                                "Controller error behind the stop: {} ({msg})",
                                servo_err_hint(code)
                            );
                        }
                        // The arm stopped following the stream. Leave servo
                        // mode so point motions like restore are not stuck
                        // behind it, the next vel command re-enters
                        info!("Servo stream lost the arm, exiting servo mode");
                        servo.active = false;
                        servo.vel = [0.0; 6];
                        servo.target = [0.0; 6];
                        let _ = unsafe { binding::servo_move_enable(handle, 0) };
                    }
                }
                Ok(())
            }
            Backend::Mock(state) => {
                state.advance();
                Ok(())
            }
        }
    }

    /// Leave servo mode and stop the arm. The mock clears its jog velocity
    fn servo_end(&mut self) -> Result<(), String> {
        match self {
            Backend::Real { handle, servo } => {
                if !servo.active {
                    return Ok(());
                }
                servo.active = false;
                servo.vel = [0.0; 6];
                servo.target = [0.0; 6];
                let ret = check("Disable servo mode", unsafe {
                    binding::servo_move_enable(handle, 0)
                });
                if ret.is_ok() {
                    info!("Servo mode disabled");
                }
                ret
            }
            Backend::Mock(state) => {
                state.advance();
                state.jog = [0.0; 6];
                Ok(())
            }
        }
    }

    /// Abort every ongoing motion
    fn abort_motion(&mut self) -> Result<(), String> {
        match self {
            Backend::Real { handle, .. } => {
                check("Abort motion", unsafe { binding::motion_abort(handle) })
            }
            Backend::Mock(state) => {
                state.advance();
                state.jog = [0.0; 6];
                Ok(())
            }
        }
    }

    /// Stop the arm and release every control mode, used on shutdown
    fn shutdown(&mut self) {
        let _ = self.servo_end();
        let _ = self.abort_motion();
    }

    /// Clear the e-stop error state
    fn clear_error(&mut self) -> Result<(), String> {
        match self {
            Backend::Real { handle, .. } => estop_clear(handle),
            Backend::Mock(state) => {
                state.estop = false;
                Ok(())
            }
        }
    }

    /// Power on and enable the robot, rejected while the e-stop is active
    fn power_on(&mut self) -> Result<(), String> {
        match self {
            Backend::Real { handle, .. } => {
                let mut st = RobotState::default();
                check("Read state", unsafe {
                    binding::get_robot_state(handle, &mut st)
                })?;
                ensure_powered_enabled(handle, &st)
            }
            Backend::Mock(state) => state.power_on(),
        }
    }

    /// Disable the servos and power off
    fn power_off(&mut self) -> Result<(), String> {
        match self {
            Backend::Real { handle, .. } => {
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
                Ok(())
            }
            Backend::Mock(state) => {
                state.power_off();
                Ok(())
            }
        }
    }

    /// Restore the built-in home pose. The real robot moves there, the mock
    /// teleports because no point motion is simulated
    async fn restore_home(&mut self) -> Result<(), String> {
        match self {
            Backend::Real { handle, .. } => restore(handle, None, 3.14).await,
            Backend::Mock(state) => {
                state.restore_home();
                Ok(())
            }
        }
    }

    /// Report the state as the protocol status JSON
    fn status_json(&mut self) -> Result<String, String> {
        match self {
            Backend::Real { handle, .. } => {
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
                    "head_rpy": [
                        tcp.rpy.rx.to_degrees(),
                        tcp.rpy.ry.to_degrees(),
                        tcp.rpy.rz.to_degrees(),
                    ],
                });
                Ok(format!("status {json}\n"))
            }
            Backend::Mock(state) => {
                state.advance();
                Ok(state.status_json())
            }
        }
    }
}

/// The TCP pose and joint angles in degrees the mock starts and resets to
const HOME_HEAD: [f64; 6] = [-50.1, -6.0, 396.8, 0.0, 90.0, 0.0];
const HOME_JOINTS: [f64; 6] = [0.0, 90.0, -90.0, 0.0, -90.0, 0.0];

/// The longest time a paused simulation may integrate in one step. A real
/// jog runs until stopped, but a mock left running for minutes must not
/// shoot across the whole workspace on the next command
const MAX_STEP: f64 = 0.5;

/// Simulated robot with the same state machine as the controller. The mock
/// jogs on wall-clock time like the real arm, so a status after a pause
/// reflects the motion that happened in between
#[derive(Clone, Copy)]
struct MockState {
    powered: bool,
    enabled: bool,
    estop: bool,
    /// TCP x y z in mm and rx ry rz in degrees
    head: [f64; 6],
    /// Joint angles in degrees, static because the mock only simulates the TCP
    joints: [f64; 6],
    /// The jog velocity of each axis in protocol units, mm/s and deg/s
    jog: [f64; 6],
    /// When the jog velocities were applied last
    last: Instant,
}

impl MockState {
    fn new() -> Self {
        Self {
            powered: false,
            enabled: false,
            estop: false,
            head: HOME_HEAD,
            joints: HOME_JOINTS,
            jog: [0.0; 6],
            last: Instant::now(),
        }
    }

    /// Integrate the jog velocities over the time since the last advance
    fn advance(&mut self) {
        let dt = self.last.elapsed().as_secs_f64().min(MAX_STEP);
        self.last = Instant::now();
        self.integrate(dt);
    }

    /// Run the simulation forward by an explicit delta, used by the tests
    #[cfg(test)]
    fn tick(&mut self, dt: f64) {
        self.last = Instant::now();
        self.integrate(dt);
    }

    fn integrate(&mut self, dt: f64) {
        for i in 0..6 {
            self.head[i] += self.jog[i] * dt;
        }
    }

    /// Check that the simulated arm can move, rejected while it cannot
    fn check_can_move(&mut self) -> Result<(), String> {
        self.advance();
        if self.estop {
            return Err("E-stop is active, clear it first".into());
        }
        if !self.enabled {
            return Err("Servos are not enabled, power on first".into());
        }
        Ok(())
    }

    fn power_on(&mut self) -> Result<(), String> {
        self.advance();
        if self.estop {
            return Err("E-stop is pressed, release the button first".into());
        }
        self.powered = true;
        self.enabled = true;
        Ok(())
    }

    fn power_off(&mut self) {
        self.advance();
        self.enabled = false;
        self.powered = false;
        self.jog = [0.0; 6];
    }

    fn restore_home(&mut self) {
        self.advance();
        self.head = HOME_HEAD;
        self.joints = HOME_JOINTS;
        self.jog = [0.0; 6];
    }

    fn status_json(&self) -> String {
        let json = serde_json::json!({
            "estop": self.estop,
            "powered": self.powered,
            "enabled": self.enabled,
            "joints": self.joints,
            "head_pos": [self.head[0], self.head[1], self.head[2]],
            "head_rpy": [self.head[3], self.head[4], self.head[5]],
        });
        format!("status {json}\n")
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
            .ok_or_else(|| format!("{} expects numeric arguments", parts[0]))
    };
    match parts[0] {
        "vel" => {
            if parts.len() != 7 {
                return Err("vel expects 6 numbers".into());
            }
            Ok(ProtoCmd::Vel(
                num(1)?,
                num(2)?,
                num(3)?,
                num(4)?,
                num(5)?,
                num(6)?,
            ))
        }
        "estop-clear" => Ok(ProtoCmd::EstopClear),
        "poweron" => Ok(ProtoCmd::PowerOn),
        "poweroff" => Ok(ProtoCmd::PowerOff),
        "reset" => Ok(ProtoCmd::Reset),
        "status" => Ok(ProtoCmd::Status),
        other => Err(format!("unknown command: {other}")),
    }
}

/// Serve the gamepad control protocol over TCP. While a client is connected
/// the servo pulse loop feeds the arm continuously, so velocity commands
/// only set the target and every axis moves at the same time. One client at
/// a time owns the arm
async fn serve(backend: Backend, port: u16) -> Result<(), String> {
    let listener = TcpListener::bind(("0.0.0.0", port))
        .await
        .map_err(|e| format!("Bind port {port} failed: {e}"))?;
    if backend.is_mock() {
        info!("Mock control server listening on port {port}, no controller involved");
    } else {
        info!("Control server listening on port {port}");
    }
    // The backend is shared between the accept loop and the client tasks
    let shared = Arc::new(Mutex::new(backend));
    // All client tasks, aborted and joined on shutdown
    let mut clients: Vec<JoinHandle<()>> = Vec::new();
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("Ctrl+C received, stopping all motions and disconnecting clients");
                // Stop the arm and leave servo mode before killing the tasks
                let mut backend = shared.lock().await;
                backend.shutdown();
                drop(backend);
                for c in &clients {
                    c.abort();
                }
                for c in clients {
                    let _ = c.await;
                }
                break;
            }
            accepted = listener.accept() => {
                let (stream, addr) = match accepted {
                    Ok(x) => x,
                    Err(e) => {
                        warn!("Accept failed: {e}");
                        continue;
                    }
                };
                // Drop the handles of finished clients before the count check
                clients.retain(|c| !c.is_finished());
                if !clients.is_empty() {
                    warn!("Another control client is connected, rejecting {addr}");
                    continue;
                }
                info!("Control client connected from {addr}");
                let shared = shared.clone();
                clients.push(tokio::spawn(async move {
                    if let Err(e) = handle_client(shared, stream).await {
                        warn!("Control client {addr} error: {e}");
                    }
                }));
            }
        }
    }
    info!("Control server stopped");
    Ok(())
}

/// Serve one control client: read protocol lines and run the servo pulse
/// loop while the client is connected
async fn handle_client(shared: Arc<Mutex<Backend>>, stream: TcpStream) -> Result<(), String> {
    let (reader, mut writer) = stream.into_split();
    let mut lines = tokio::io::BufReader::new(reader).lines();
    // The pulse loop feeds the arm while it has a velocity target
    let mut ticker = tokio::time::interval(SERVO_PERIOD);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut last_pulse = Instant::now();
    let mut idle = 0.0;
    loop {
        tokio::select! {
            line = lines.next_line() => {
                let line = match line {
                    Ok(Some(l)) => l,
                    Ok(None) => break,
                    Err(e) => return Err(format!("Read line failed: {e}")),
                };
                let line = line.trim();
                if line.is_empty() {
                    continue;
                }
                let mut backend = shared.lock().await;
                if backend.is_mock() {
                    info!("[mock] {line}");
                }
                let reply = handle_proto(&mut backend, line).await;
                drop(backend);
                writer
                    .write_all(reply.as_bytes())
                    .await
                    .map_err(|e| format!("Write reply failed: {e}"))?;
            }
            _ = ticker.tick() => {
                // The mock integrates on demand, only the real arm is pulsed
                if shared.lock().await.is_mock() {
                    continue;
                }
                let now = Instant::now();
                let dt = now.duration_since(last_pulse).as_secs_f64().min(MAX_PULSE_DT);
                last_pulse = now;
                let mut backend = shared.lock().await;
                if backend.target() == [0.0; 6] {
                    // Keep the servo stream alive briefly, then leave servo
                    // mode so a resting client does not occupy the arm
                    idle += dt;
                    if idle >= IDLE_END {
                        let _ = backend.servo_end();
                        idle = 0.0;
                    } else if let Err(e) = backend.pulse(dt) {
                        warn!("Servo pulse failed: {e}");
                        let _ = backend.servo_end();
                    }
                } else {
                    idle = 0.0;
                    if let Err(e) = backend.pulse(dt) {
                        warn!("Servo pulse failed: {e}");
                        let _ = backend.servo_end();
                    }
                }
                drop(backend);
            }
        }
    }
    // Disconnecting stops the arm and leaves servo mode
    let mut backend = shared.lock().await;
    let _ = backend.servo_end();
    info!("Control client disconnected");
    Ok(())
}

/// Execute one protocol command and return the reply line
async fn handle_proto(backend: &mut Backend, line: &str) -> String {
    let cmd = match parse_proto(line) {
        Ok(c) => c,
        Err(e) => return format!("err {e}\n"),
    };
    match exec_cmd(backend, cmd).await {
        Ok(reply) => reply,
        Err(e) => format!("err {e}\n"),
    }
}

/// Run one parsed protocol command against either backend. A vel command
/// only sets the target velocity of the pulse loop, so every axis moves at
/// the same time and the mock follows the same motion model
async fn exec_cmd(backend: &mut Backend, cmd: ProtoCmd) -> Result<String, String> {
    match cmd {
        ProtoCmd::Vel(dx, dy, dz, drx, dry, drz) => {
            let target = clamp_vel([dx, dy, dz, drx, dry, drz]);
            // The first motion command enters servo mode, rejected while
            // the arm cannot move
            if target != [0.0; 6] {
                backend.ensure_servo()?;
            }
            backend.set_target(target);
            Ok("ok\n".into())
        }
        ProtoCmd::EstopClear => {
            // Leave servo mode first, it may already be gone after the e-stop
            let _ = backend.servo_end();
            // Report the controller error before it is cleared, the e-stop
            // button is the recovery step after a stopped servo stream
            if let Backend::Real { handle, .. } = backend {
                if let Some((code, msg)) = read_robot_error(handle) {
                    warn!(
                        "Controller error before clear: {} ({msg})",
                        servo_err_hint(code)
                    );
                }
            }
            backend.clear_error()?;
            Ok("ok\n".into())
        }
        ProtoCmd::PowerOn => {
            let _ = backend.servo_end();
            if backend.in_estop()? {
                return Err("E-stop is pressed, release the button first".into());
            }
            backend.power_on()?;
            Ok("ok\n".into())
        }
        ProtoCmd::PowerOff => {
            let _ = backend.servo_end();
            backend.power_off()?;
            Ok("ok\n".into())
        }
        ProtoCmd::Reset => {
            let _ = backend.servo_end();
            if backend.in_estop()? {
                return Err("E-stop is pressed, release the button first".into());
            }
            backend.power_on()?;
            backend.restore_home().await?;
            Ok("ok\n".into())
        }
        ProtoCmd::Status => backend.status_json(),
    }
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
        (-130.0, 130.0), // J3, JAKA Mini (not 360, verified in the URDF)
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

/// Compare two poses within the FK tolerance. Positions are compared in mm
/// and orientations as the matrix rotation angle, because the rpy values of
/// an equivalent orientation jump near the gimbal lock
fn pose_close(a: &CartesianPose, b: &CartesianPose) -> bool {
    const POS_TOL: f64 = 1.0; // mm
    const ORI_TOL: f64 = 5.0_f64.to_radians(); // degrees
    let pos = ((a.tran.x - b.tran.x).powi(2)
        + (a.tran.y - b.tran.y).powi(2)
        + (a.tran.z - b.tran.z).powi(2))
    .sqrt();
    let ra = rpy_to_rot(a.rpy.rx, a.rpy.ry, a.rpy.rz);
    let rb = rpy_to_rot(b.rpy.rx, b.rpy.ry, b.rpy.rz);
    pos < POS_TOL && rot_angle(ra, rb) < ORI_TOL
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
    use super::Backend;
    use super::MockState;
    use super::ProtoCmd;
    use super::exec_cmd;
    use super::parse_proto;

    #[test]
    fn parse_vel_command() {
        match parse_proto("vel 10 -5 2.5 0 0 -15").unwrap() {
            ProtoCmd::Vel(dx, dy, dz, drx, dry, drz) => {
                assert_eq!((dx, dy, dz), (10.0, -5.0, 2.5));
                assert_eq!((drx, dry, drz), (0.0, 0.0, -15.0));
            }
            _ => panic!("expected vel"),
        }
    }

    #[test]
    fn parse_simple_commands() {
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
        assert!(parse_proto("vel 1 2").is_err());
        assert!(parse_proto("vel a b c d e f").is_err());
        assert!(parse_proto("move 1 2 3").is_err());
    }

    #[tokio::test]
    async fn mock_vel_requires_power_and_clamps() {
        let mut backend = Backend::Mock(MockState::new());
        // The real arm rejects motion before power on, the mock does the same
        assert!(
            exec_cmd(&mut backend, ProtoCmd::Vel(10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .await
                .is_err()
        );
        assert!(exec_cmd(&mut backend, ProtoCmd::PowerOn).await.is_ok());
        assert!(
            exec_cmd(&mut backend, ProtoCmd::Vel(10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .await
                .is_ok()
        );
        // Velocities above the safe limits get clamped, here 1000 to 100 mm/s
        assert!(
            exec_cmd(&mut backend, ProtoCmd::Vel(1000.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .await
                .is_ok()
        );
        let st = match &backend {
            Backend::Mock(st) => st,
            Backend::Real { .. } => unreachable!(),
        };
        assert_eq!(st.jog[0], 100.0);
        // A zero velocity stops the axis
        assert!(
            exec_cmd(&mut backend, ProtoCmd::Vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .await
                .is_ok()
        );
        let st = match &backend {
            Backend::Mock(st) => st,
            Backend::Real { .. } => unreachable!(),
        };
        assert_eq!(st.jog[0], 0.0);
    }

    #[tokio::test]
    async fn mock_estop_blocks_motion_until_cleared() {
        let mut backend = Backend::Mock(MockState::new());
        assert!(exec_cmd(&mut backend, ProtoCmd::PowerOn).await.is_ok());
        // Press the e-stop the way the physical button would
        match &mut backend {
            Backend::Mock(st) => st.estop = true,
            Backend::Real { .. } => unreachable!(),
        }
        assert!(
            exec_cmd(&mut backend, ProtoCmd::Vel(10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .await
                .is_err()
        );
        assert!(exec_cmd(&mut backend, ProtoCmd::PowerOn).await.is_err());
        assert!(exec_cmd(&mut backend, ProtoCmd::EstopClear).await.is_ok());
        assert!(exec_cmd(&mut backend, ProtoCmd::PowerOn).await.is_ok());
        assert!(
            exec_cmd(&mut backend, ProtoCmd::Vel(10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .await
                .is_ok()
        );
        let st = match &backend {
            Backend::Mock(st) => st,
            Backend::Real { .. } => unreachable!(),
        };
        assert!(st.powered && st.enabled && !st.estop);
        assert_eq!(st.jog[0], 10.0);
    }

    #[tokio::test]
    async fn mock_reset_returns_home_and_stays_on() {
        let mut backend = Backend::Mock(MockState::new());
        assert!(exec_cmd(&mut backend, ProtoCmd::PowerOn).await.is_ok());
        assert!(
            exec_cmd(&mut backend, ProtoCmd::Vel(10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                .await
                .is_ok()
        );
        assert!(exec_cmd(&mut backend, ProtoCmd::Reset).await.is_ok());
        let st = match &backend {
            Backend::Mock(st) => st,
            Backend::Real { .. } => unreachable!(),
        };
        assert!(st.powered && st.enabled);
        assert_eq!(st.head, super::HOME_HEAD);
        assert_eq!(st.jog, [0.0; 6]);
    }

    #[test]
    fn mock_jogs_on_wall_clock() {
        let mut st = MockState::new();
        st.power_on().unwrap();
        st.jog = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        st.tick(0.5);
        // The wall clock may add microseconds of motion on top of the tick
        assert!((st.head[0] - (-0.1)).abs() < 1e-6);
        st.jog = [0.0; 6];
        let before = st.head[0];
        st.tick(1.0);
        assert_eq!(st.head[0], before);
    }

    #[test]
    fn rot_rpy_roundtrip() {
        use super::{rot_angle, rot_to_rpy, rpy_to_rot};
        let cases = [
            (0.1, 0.2, -0.3),
            (0.0, 1.5707, 0.5),
            (-1.2, 1.55, 0.7),
            (0.0, -1.5707, -0.4),
            (2.9, 0.0, -1.5),
        ];
        for (rx, ry, rz) in cases {
            let m = rpy_to_rot(rx, ry, rz);
            let (a, b, c) = rot_to_rpy(m);
            let back = rpy_to_rot(a, b, c);
            assert!(rot_angle(m, back) < 1e-6, "{rx},{ry},{rz} -> {a},{b},{c}");
        }
    }

    #[test]
    fn rot_z_accumulation_through_gimbal_lock() {
        use super::{mat_mul, rot_angle, rot_to_rpy, rot_z, rpy_to_rot};
        // Rotate about base Z in small steps starting at the lock point
        let mut rot = rpy_to_rot(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        let step = 1.0_f64.to_radians();
        for _ in 0..90 {
            rot = mat_mul(rot_z(step), rot);
            let (rx, ry, rz) = rot_to_rpy(rot);
            let back = rpy_to_rot(rx, ry, rz);
            assert!(rot_angle(rot, back) < 1e-6);
            // The extracted rpy may jump near the lock, but stays finite
            assert!(rx.is_finite() && ry.is_finite() && rz.is_finite());
        }
    }
}
