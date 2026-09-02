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
/// Stick deflection to millimeters per frame, the loop runs at 20 Hz
const MOVE_SCALE: f32 = 4.0;
/// Stick deflection to degrees per frame
const ROTATE_SCALE: f32 = 3.0;
/// Trigger travel to millimeters per frame
const TRIGGER_SCALE: f32 = 4.0;

/// The input snapshot of one control frame
#[derive(Default)]
struct Input {
    stick_x: f32,
    stick_y: f32,
    rot_x: f32,
    rot_y: f32,
    trigger: f32,
    cross: bool,
    circle: bool,
    ps: bool,
    select: bool,
}

fn read_input(gp: &Gamepad) -> Input {
    Input {
        stick_x: gp.value(Axis::LeftStickX),
        stick_y: gp.value(Axis::LeftStickY),
        rot_x: gp.value(Axis::RightStickX),
        rot_y: gp.value(Axis::RightStickY),
        // LeftZ lowers, RightZ raises
        trigger: gp.value(Axis::RightZ) - gp.value(Axis::LeftZ),
        cross: gp.is_pressed(Button::South),
        circle: gp.is_pressed(Button::East),
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

        // Motion commands stream every frame while a stick or trigger is
        // deflected, releasing it stops the stream and the robot holds
        let dx = if inp.stick_x.abs() > DEADZONE {
            inp.stick_x * MOVE_SCALE
        } else {
            0.0
        };
        let dy = if inp.stick_y.abs() > DEADZONE {
            inp.stick_y * MOVE_SCALE
        } else {
            0.0
        };
        if dx != 0.0 || dy != 0.0 {
            send(&mut stream, &mut reader, &format!("move {dx:.1} {dy:.1} 0"));
        }
        let drz = if inp.rot_x.abs() > DEADZONE {
            inp.rot_x * ROTATE_SCALE
        } else {
            0.0
        };
        let dry = if inp.rot_y.abs() > DEADZONE {
            inp.rot_y * ROTATE_SCALE
        } else {
            0.0
        };
        if drz != 0.0 || dry != 0.0 {
            send(
                &mut stream,
                &mut reader,
                &format!("rotate 0 {dry:.1} {drz:.1}"),
            );
        }
        if inp.trigger.abs() > 0.1 {
            send(
                &mut stream,
                &mut reader,
                &format!("move 0 0 {:.1}", inp.trigger * TRIGGER_SCALE),
            );
        }

        // Buttons fire once on the press edge
        if inp.cross && !prev.cross {
            send(&mut stream, &mut reader, "stop");
        }
        if inp.circle && !prev.circle {
            send(&mut stream, &mut reader, "reset");
        }
        if inp.ps && !prev.ps {
            send(&mut stream, &mut reader, "poweron");
        }
        if inp.select && !prev.select {
            send(&mut stream, &mut reader, "poweroff");
        }
        prev = inp;
        std::thread::sleep(Duration::from_millis(50));
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
