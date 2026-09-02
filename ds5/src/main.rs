//! DualSense gamepad controller for the JAKA arm.
//!
//! Reads a DualSense gamepad and streams commands to the jaka-cli serve
//! endpoint over TCP. This binary is a pure protocol layer, it never links
//! the JAKA SDK.

use gilrs::{Axis, Button, Gamepad, Gilrs};
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::time::Duration;

/// Stick deflection below this value is treated as neutral
const DEADZONE: f32 = 0.15;
/// Full stick deflection to velocity in mm/s
const MOVE_SCALE: f32 = 100.0;
/// Full stick deflection to angular velocity in deg/s
const ROTATE_SCALE: f32 = 20.0;
/// Shoulder button velocity in mm/s while held
const Z_SCALE: f32 = 40.0;

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
    ps: bool,
    select: bool,
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
        ps: gp.is_pressed(Button::Mode),
        select: gp.is_pressed(Button::Select),
    }
}

fn main() {
    let (ip, port) = parse_args();
    let mut stream = TcpStream::connect((ip.as_str(), port))
        .unwrap_or_else(|e| die(&format!("connect {ip}:{port} failed: {e}")));
    let mut reader = BufReader::new(stream.try_clone().expect("clone the stream"));
    println!("connected to {ip}:{port}");

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
        // Send only the changes: a jog axis keeps its velocity until the
        // next command, so a steady stick must not resend the same value
        if vel != prev_vel {
            send(
                &mut stream,
                &mut reader,
                &format!(
                    "vel {:.0} {:.0} {:.0} {:.0} {:.0} {:.0}",
                    vel[0], vel[1], vel[2], vel[3], vel[4], vel[5]
                ),
            );
            prev_vel = vel;
        }
        // Buttons fire once on the press edge
        if inp.cross && !prev.cross {
            send(&mut stream, &mut reader, "estop-clear");
        }
        if inp.triangle && !prev.triangle {
            send(&mut stream, &mut reader, "reset");
        }
        if inp.ps && !prev.ps {
            send(&mut stream, &mut reader, "poweron");
        }
        if inp.select && !prev.select {
            send(&mut stream, &mut reader, "poweroff");
        }
        prev = inp;
        // Scan at 100 Hz, only changed velocities produce a command
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Send one protocol line and read the reply, exits when the link fails
fn send(stream: &mut TcpStream, reader: &mut BufReader<TcpStream>, cmd: &str) {
    if stream.write_all(cmd.as_bytes()).is_err() || stream.write_all(b"\n").is_err() {
        die("write failed, is jaka-cli serve running?");
    }
    let mut line = String::new();
    match reader.read_line(&mut line) {
        Ok(0) | Err(_) => die("read failed, is jaka-cli serve running?"),
        Ok(_) => {}
    }
    let reply = line.trim();
    if reply != "ok" {
        eprintln!("{cmd}: {reply}");
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
