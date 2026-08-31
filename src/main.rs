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

use binding::{BOOL, JKHD, JointValue, MoveMode, OptionalCond, RobotState, check};
use clap::Parser;
use cli::{Cli, Command};
use log::{error, info, warn};
use serde::Deserialize;

/// Joint angles document as recorded by inspect
#[derive(Deserialize)]
struct JointsDoc {
    joints: Vec<f64>,
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
            "No subcommand given. Expected one of status, power-on, power-off, estop-clear, inspect, rot, restore. Use --help for usage"
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
            info!(
                "[dry-run] Will connect to controller {} and restore joints from {}",
                args.ip,
                file.display()
            );
            info!("[dry-run] Speed {:.2} rad/s", speed);
        }
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
        Command::Inspect => {
            let mut cur = JointValue::zero();
            check("Read joint position", unsafe {
                binding::get_joint_position(handle, &mut cur)
            })?;
            print_joints_json(stdout_fd, &cur);
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
        Command::PowerOn | Command::Rot { .. } | Command::Restore { .. } => {
            if st.estoped != 0 {
                return Err(
                    "E-stop is pressed. Release the button physically, then run estop-clear".into(),
                );
            }
            ensure_powered_enabled(handle, &st)?;
            match command {
                Command::Rot { joint, deg, speed } => rot(handle, *joint, *deg, *speed).await,
                Command::Restore { file, speed } => restore(handle, file, *speed).await,
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

    run_blocking_move(handle, target, MoveMode::Incr, speed).await?;

    // Read the final joint angles
    let mut fin = JointValue::zero();
    check("Read final joint position", unsafe {
        binding::get_joint_position(handle, &mut fin)
    })?;
    info!("Final joint angles in degrees: {}", format_joints(&fin));
    info!("Done. The arm completed a full rotation");
    Ok(())
}

/// Restore the joint angles recorded by inspect from a JSON file
async fn restore(handle: &JKHD, file: &Path, speed: f64) -> Result<(), String> {
    // Load and parse the recorded joints, stored in degrees
    let text = std::fs::read_to_string(file)
        .map_err(|e| format!("Failed to read {}: {e}", file.display()))?;
    let doc: JointsDoc = serde_json::from_str(&text)
        .map_err(|e| format!("Invalid JSON in {}: {e}", file.display()))?;
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
        "Restoring joints from {} at {:.2} rad/s",
        file.display(),
        speed
    );
    run_blocking_move(handle, target, MoveMode::Abs, speed).await?;

    // Read the final joint angles
    let mut fin = JointValue::zero();
    check("Read final joint position", unsafe {
        binding::get_joint_position(handle, &mut fin)
    })?;
    info!("Final joint angles in degrees: {}", format_joints(&fin));
    info!("Restore complete");
    Ok(())
}

/// Run a blocking SDK joint move with Ctrl+C abort support
async fn run_blocking_move(
    handle: &JKHD,
    target: JointValue,
    mode: MoveMode,
    speed: f64,
) -> Result<(), String> {
    // The SDK move call blocks the calling thread. Run it on a blocking task so
    // the runtime stays responsive to the Ctrl+C signal
    let h = *handle;
    let mut motion = tokio::task::spawn_blocking(move || unsafe {
        binding::joint_move_extend(
            &h,
            &target,
            mode,
            1, // is_block: block until the motion completes
            speed,
            1.0, // acc, rad/s^2
            0.0, // tol
            std::ptr::null::<OptionalCond>(),
        )
    });

    let outcome = tokio::select! {
        ret = &mut motion => match ret {
            Ok(code) => check("Joint move", code),
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
            let _ = tokio::time::timeout(Duration::from_secs(5), &mut motion).await;
            info!("Motion aborted by Ctrl+C");
            return Ok(());
        }
    };
    outcome
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

/// Print the joint angles in degrees as a JSON object on the original stdout
fn print_joints_json(fd: i32, j: &JointValue) {
    let joints: Vec<f64> = j.j_val.iter().map(|v| v.to_degrees()).collect();
    let json = serde_json::json!({ "joints": joints }).to_string();
    binding::write_to_fd(fd, &format!("{json}\n"));
}
