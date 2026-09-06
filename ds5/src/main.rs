//! DualSense gamepad controller for the JAKA arm.
//!
//! Reads a DualSense gamepad and streams commands to the jaka-cli serve
//! endpoint over TCP. This binary is a pure protocol layer, it never links
//! the JAKA SDK.

use gilrs::{Axis, Button, Gamepad, Gilrs};
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::thread;
use std::time::{Duration, Instant};

/// Stick deflection below this value is treated as neutral
const DEADZONE: f32 = 0.15;
/// Full stick deflection to velocity in mm/s
const MOVE_SCALE: f32 = 100.0;
/// Full stick deflection to angular velocity in deg/s
const ROTATE_SCALE: f32 = 20.0;
/// Shoulder button velocity in mm/s while held
const Z_SCALE: f32 = 40.0;
/// Right trigger level that turns the tool outputs on
const SUCTION_LEVEL: f32 = 0.5;
/// How often a steady velocity is resent, so the serve side recovers when
/// its servo stream stopped on an error
const HEARTBEAT: Duration = Duration::from_millis(500);

/// Quantize a velocity to whole units and snap tiny values to zero, so a
/// resting stick sends a stable command instead of jittering
fn quant(v: f32) -> f32 {
    let q = v.round();
    if q.abs() < 1.0 { 0.0 } else { q }
}

/// The input snapshot of one control frame
#[derive(Default)]
struct Input {
    stick_x: f32,
    stick_y: f32,
    rot_x: f32,
    rot_y: f32,
    shoulder_l: bool,
    shoulder_r: bool,
    cross: bool,
    triangle: bool,
    dpad_up: bool,
    dpad_down: bool,
    /// Right trigger level, 0 to 1, drives the tool outputs past half
    r2: f32,
}

fn read_input(gp: &Gamepad) -> Input {
    Input {
        stick_x: gp.value(Axis::LeftStickX),
        stick_y: gp.value(Axis::LeftStickY),
        rot_x: gp.value(Axis::RightStickX),
        rot_y: gp.value(Axis::RightStickY),
        shoulder_l: gp.is_pressed(Button::LeftTrigger),
        shoulder_r: gp.is_pressed(Button::RightTrigger),
        cross: gp.is_pressed(Button::South),
        triangle: gp.is_pressed(Button::North),
        dpad_up: gp.is_pressed(Button::DPadUp),
        dpad_down: gp.is_pressed(Button::DPadDown),
        // The analog triggers arrive as button events carrying the axis
        // level, 0 when released and 1 when fully pressed
        r2: gp
            .button_data(Button::RightTrigger2)
            .map(|d| d.value())
            .unwrap_or(0.0),
    }
}

fn main() {
    let (ip, port) = parse_args();
    let mut stream = TcpStream::connect((ip.as_str(), port))
        .unwrap_or_else(|e| die(&format!("connect {ip}:{port} failed: {e}")));
    println!("connected to {ip}:{port}");

    // A background thread consumes the replies so the control loop never
    // blocks on the network round trip. Non-ok replies surface serve side
    // errors, a closed connection exits the program
    let reply_stream = stream.try_clone().expect("clone the stream");
    thread::spawn(move || {
        let mut lines = BufReader::new(reply_stream).lines();
        loop {
            let line = match lines.next() {
                Some(Ok(l)) => l,
                // The stream ended or failed, the serve side is gone
                None | Some(Err(_)) => die("serve connection closed"),
            };
            let reply = line.trim();
            if reply.is_empty() {
                continue;
            }
            if reply != "ok" {
                eprintln!("serve: {reply}");
            }
        }
    });

    let mut gilrs = Gilrs::new().unwrap_or_else(|e| die(&format!("gamepad init failed: {e}")));
    let gp_id = gilrs
        .gamepads()
        .next()
        .map(|(id, _)| id)
        .unwrap_or_else(|| die("no gamepad connected, plug in a DualSense first"));
    println!("gamepad: {}", gilrs.gamepad(gp_id).name());

    let mut prev = Input::default();
    // The velocity of the last sent vel command, NaN forces the first frame
    let mut prev_vel = [f32::NAN; 6];
    let mut last_sent = Instant::now();
    // The suction state, driven by the right trigger past the half level
    let mut prev_suck = false;
    loop {
        // Process the pending events. The gamepad mapping is only applied
        // and the cached state only updated inside next_event, without it
        // every axis reads as zero
        while gilrs.next_event().is_some() {}
        let gp = gilrs.gamepad(gp_id);
        if !gp.is_connected() {
            die("gamepad disconnected");
        }
        let inp = read_input(&gp);

        // The right stick tilts the head around X and turns it around Z
        let dx = if inp.stick_x.abs() > DEADZONE {
            quant(inp.stick_x * MOVE_SCALE)
        } else {
            0.0
        };
        let dy = if inp.stick_y.abs() > DEADZONE {
            quant(inp.stick_y * MOVE_SCALE)
        } else {
            0.0
        };
        // L1 lowers, R1 raises
        let dz = if inp.shoulder_l {
            -Z_SCALE
        } else if inp.shoulder_r {
            Z_SCALE
        } else {
            0.0
        };
        let drx = if inp.rot_y.abs() > DEADZONE {
            quant(inp.rot_y * ROTATE_SCALE)
        } else {
            0.0
        };
        let drz = if inp.rot_x.abs() > DEADZONE {
            quant(inp.rot_x * ROTATE_SCALE)
        } else {
            0.0
        };
        let vel = [dx, dy, dz, drx, 0.0, drz];
        // All inputs merge into one velocity command, so moving, lifting and
        // rotating happen at the same time. Send only the changes: a jog axis
        // keeps its velocity until the next command, so a steady stick must
        // not resend the same value
        if vel != prev_vel {
            send_vel(&mut stream, vel);
            prev_vel = vel;
            last_sent = Instant::now();
        } else if vel != [0.0; 6] && last_sent.elapsed() >= HEARTBEAT {
            // The serve side may have stopped its servo stream on an error,
            // resend the running velocity so it re-enters servo mode
            send_vel(&mut stream, vel);
            last_sent = Instant::now();
        }
        // Buttons fire once on the press edge
        if inp.cross && !prev.cross {
            write_cmd(&mut stream, "estop-clear");
        }
        if inp.triangle && !prev.triangle {
            write_cmd(&mut stream, "reset");
        }
        if inp.dpad_up && !prev.dpad_up {
            write_cmd(&mut stream, "poweron");
        }
        if inp.dpad_down && !prev.dpad_down {
            write_cmd(&mut stream, "poweroff");
        }
        // The right trigger past half drives both tool outputs, the typical
        // valve wiring of a suction cup
        let suck = inp.r2 >= SUCTION_LEVEL;
        if suck != prev_suck {
            let state = if suck { "on" } else { "off" };
            for index in 0..2 {
                write_cmd(&mut stream, &format!("do tool {index} {state}"));
            }
            prev_suck = suck;
        }
        prev = inp;
        // Scan at 100 Hz, only changed velocities produce a command
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Send one velocity command
fn send_vel(stream: &mut TcpStream, vel: [f32; 6]) {
    write_cmd(
        stream,
        &format!(
            "vel {:.0} {:.0} {:.0} {:.0} {:.0} {:.0}",
            vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]
        ),
    );
}

/// Send one protocol line without waiting for the reply, exits on write
/// failure. The reply thread reports serve side errors
fn write_cmd(stream: &mut TcpStream, cmd: &str) {
    if stream.write_all(cmd.as_bytes()).is_err() || stream.write_all(b"\n").is_err() {
        die("write failed, is jaka-cli serve running?");
    }
}

fn parse_args() -> (String, u16) {
    let mut ip = "127.0.0.1".to_string();
    let mut port = 5533;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--ip" => ip = args.next().expect("--ip needs a value"),
            "--port" => {
                port = args
                    .next()
                    .expect("--port needs a value")
                    .parse()
                    .expect("port must be a number")
            }
            "--help" | "-h" => {
                print!("{}", include_str!("jaka_ds5_control_help.txt"));
                std::process::exit(0);
            }
            other => die(&format!("unknown argument: {other}")),
        }
    }
    (ip, port)
}

fn die(msg: &str) -> ! {
    eprintln!("{msg}");
    std::process::exit(1);
}
