//! Tier 3 Rust bring-up: normalized gripper opening control.
//!
//! Opening `0.0` is fully closed and opening `1.0` is fully open. `Robot::enable`
//! runs opening calibration before `open`, `set_opening`, and `close` are used.

use std::env;
use std::io::{self, Write};
use std::process::ExitCode;
use std::sync::Mutex;
use std::thread;
use std::time::{Duration, Instant};

use can_motor_control::{
    BusCapabilities, CanBus, CanFrame, CodecError, Command, Event, GripperOpeningSpec, Limits,
    MockCanBus, MotorCodec, MotorRef, MotorSpec, MotorTypeId, OpeningDirection, RobotBuilder,
};
use damiao_codec::{parse_motor_type, DamiaoCodec};

mod support;
use support::open_native_transport;

const DEFAULT_INTERFACE: &str = "can0";
const DEFAULT_SEND_ID: u32 = 0x08;
const DEFAULT_RECV_ID: u32 = 0x18;
const DEFAULT_MOTOR_TYPE: &str = "DM4310";
const DEFAULT_SECONDS: f64 = 2.0;
const DEFAULT_DEADLINE_US: u64 = 500;
const DEFAULT_CURRENT: f64 = 0.15;

struct MockFeedbackCodec {
    recv_id: u32,
    position: Mutex<f64>,
}

impl MockFeedbackCodec {
    fn new(recv_id: u32) -> Self {
        Self {
            recv_id,
            position: Mutex::new(0.0),
        }
    }
}

impl MotorCodec for MockFeedbackCodec {
    fn vendor_name(&self) -> &'static str {
        "mock-feedback"
    }

    fn supports(&self, t: MotorTypeId) -> bool {
        matches!(t, MotorTypeId::Damiao(_))
    }

    fn limits(&self, _: MotorTypeId) -> Result<Limits, CodecError> {
        Ok(Limits {
            p_max: 1.0,
            v_max: 1.0,
            t_max: 1.0,
        })
    }

    fn bind_to_bus(&mut self, _: BusCapabilities) {}

    fn encode_enable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        CanFrame::classical(m.send_id, &[0xFC])
            .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
    }

    fn encode_disable(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        CanFrame::classical(m.send_id, &[0xFD])
            .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
    }

    fn encode_set_zero(&self, m: MotorRef<'_>) -> Result<CanFrame, CodecError> {
        if let Ok(mut position) = self.position.lock() {
            *position = 0.0;
        }
        CanFrame::classical(m.recv_id, &[0xFE])
            .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
    }

    fn encode_command(&self, m: MotorRef<'_>, command: &Command) -> Result<CanFrame, CodecError> {
        if let Command::PosForce { q, .. } = *command {
            if let Ok(mut position) = self.position.lock() {
                *position = q;
            }
            CanFrame::classical(m.recv_id, &[0x55])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        } else {
            CanFrame::classical(m.send_id, &[0x55])
                .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
        }
    }

    fn encode_refresh(&self, m: MotorRef<'_>) -> Result<Option<CanFrame>, CodecError> {
        CanFrame::classical(m.recv_id, &[0xCC])
            .map(Some)
            .map_err(|_| CodecError::DecodeFailed { reason: "frame" })
    }

    fn decode(
        &self,
        frame: &CanFrame,
        _motor: motor_codec::MotorRef<'_>,
    ) -> Result<Option<Event>, CodecError> {
        if frame.id != self.recv_id {
            return Ok(None);
        }
        let q = *self.position.lock().map_err(|_| CodecError::DecodeFailed {
            reason: "mock lock",
        })?;
        Ok(Some(Event::State {
            motor_id: self.recv_id,
            q,
            dq: 0.0,
            tau: 0.0,
            t_mos: 30,
            t_rotor: 35,
        }))
    }
}

struct Args {
    mock: bool,
    interface: String,
    fd: bool,
    send_id: u32,
    recv_id: u32,
    motor_type: String,
    seconds: f64,
    deadline_us: u64,
    opening_direction: OpeningDirection,
    default_current: f64,
    current: Option<f64>,
    calibrate_only: bool,
    acknowledged: bool,
}

fn print_help() {
    println!(
        "Tier 3 Rust bring-up: normalized gripper opening control.\n\
\n\
Usage:\n\
  cargo run -p can-motor-control --example 05_gripper_opening_control -- [options]\n\
\n\
Options:\n\
  --mock                                  Use MockCanBus (no hardware)\n\
  --interface <iface>                     SocketCAN interface name (default: can0)\n\
  --fd                                    Open the bus in CAN-FD mode\n\
  --send-id <id>                          CAN id host->motor (default: 0x08)\n\
  --recv-id <id>                          CAN id motor->host (default: 0x18)\n\
  --motor-type <sku>                      Damiao motor model (default: DM4310)\n\
  --opening-direction <increasing_position|decreasing_position>\n\
  --default-current <value>               Default opening current per-unit (default: 0.15)\n\
  --current <value>                       Optional midpoint command current override\n\
  --calibrate-only                        Run calibration, print state, disable, and exit\n\
  --seconds <seconds>                     Per-opening duration, >0 and <=10\n\
  --deadline-us <us>                      Per tick bus deadline in microseconds\n\
  --i-understand-this-moves-the-gripper   Required safety acknowledgement\n\
  -h, --help                              Print this help text"
    );
}

fn parse_can_id(text: &str) -> Result<u32, String> {
    let trimmed = text.trim();
    if let Some(hex) = trimmed
        .strip_prefix("0x")
        .or_else(|| trimmed.strip_prefix("0X"))
    {
        u32::from_str_radix(hex, 16).map_err(|_| format!("invalid CAN id: {text}"))
    } else {
        trimmed
            .parse::<u32>()
            .map_err(|_| format!("invalid CAN id: {text}"))
    }
}

fn next_value(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, String> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a value"))
}

fn parse_f64(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<f64, String> {
    let value = next_value(args, flag)?;
    value
        .parse::<f64>()
        .map_err(|_| format!("invalid {flag} value: {value}"))
}

fn parse_opening_direction(text: &str) -> Result<OpeningDirection, String> {
    match text {
        "increasing_position" => Ok(OpeningDirection::IncreasingPosition),
        "decreasing_position" => Ok(OpeningDirection::DecreasingPosition),
        _ => Err(format!(
            "unknown --opening-direction {text}; expected increasing_position or decreasing_position"
        )),
    }
}

fn parse_args() -> Result<Option<Args>, String> {
    let mut parsed = Args {
        mock: false,
        interface: DEFAULT_INTERFACE.to_string(),
        fd: false,
        send_id: DEFAULT_SEND_ID,
        recv_id: DEFAULT_RECV_ID,
        motor_type: DEFAULT_MOTOR_TYPE.to_string(),
        seconds: DEFAULT_SECONDS,
        deadline_us: DEFAULT_DEADLINE_US,
        opening_direction: OpeningDirection::IncreasingPosition,
        default_current: DEFAULT_CURRENT,
        current: None,
        calibrate_only: false,
        acknowledged: false,
    };
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => return Ok(None),
            "--mock" => parsed.mock = true,
            "--fd" => parsed.fd = true,
            "--interface" => parsed.interface = next_value(&mut args, "--interface")?,
            "--send-id" => parsed.send_id = parse_can_id(&next_value(&mut args, "--send-id")?)?,
            "--recv-id" => parsed.recv_id = parse_can_id(&next_value(&mut args, "--recv-id")?)?,
            "--motor-type" => parsed.motor_type = next_value(&mut args, "--motor-type")?,
            "--seconds" => parsed.seconds = parse_f64(&mut args, "--seconds")?,
            "--deadline-us" => {
                let value = next_value(&mut args, "--deadline-us")?;
                parsed.deadline_us = value
                    .parse::<u64>()
                    .map_err(|_| format!("invalid --deadline-us value: {value}"))?;
            }
            "--opening-direction" => {
                parsed.opening_direction =
                    parse_opening_direction(&next_value(&mut args, "--opening-direction")?)?;
            }
            "--default-current" => {
                parsed.default_current = parse_f64(&mut args, "--default-current")?
            }
            "--current" => parsed.current = Some(parse_f64(&mut args, "--current")?),
            "--calibrate-only" => parsed.calibrate_only = true,
            "--i-understand-this-moves-the-gripper" => parsed.acknowledged = true,
            _ => return Err(format!("unknown argument: {arg}")),
        }
    }
    if !parsed.acknowledged {
        return Err("pass --i-understand-this-moves-the-gripper to run this example".to_string());
    }
    if parsed.seconds <= 0.0 || parsed.seconds > 10.0 {
        return Err(format!(
            "--seconds must be > 0 and <= 10, got {}",
            parsed.seconds
        ));
    }
    if parsed.default_current <= 0.0 || parsed.default_current > 1.0 {
        return Err("--default-current must be > 0.0 and <= 1.0".to_string());
    }
    if let Some(current) = parsed.current {
        if current <= 0.0 || current > 1.0 {
            return Err("--current must be > 0.0 and <= 1.0".to_string());
        }
    }
    Ok(Some(parsed))
}

fn gripper_position(robot: &can_motor_control::Robot) -> Result<f64, Box<dyn std::error::Error>> {
    let motor = robot
        .group("grip")
        .and_then(|group| group.as_gripper())
        .map(|gripper| gripper.motor())
        .ok_or("missing gripper group")?;
    Ok(motor.position())
}

fn print_gripper_state(
    robot: &can_motor_control::Robot,
    label: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let motor = robot
        .group("grip")
        .and_then(|group| group.as_gripper())
        .map(|gripper| gripper.motor())
        .ok_or("missing gripper group")?;
    println!(
        "  [{label}] pos={:+.5} vel={:+.5} tau={:+.5} t_mos={} t_rotor={}",
        motor.position(),
        motor.velocity(),
        motor.torque(),
        motor.temperature_mos(),
        motor.temperature_rotor()
    );
    Ok(())
}

fn drive_opening_for(
    robot: &mut can_motor_control::Robot,
    label: &str,
    opening: f64,
    current: Option<f64>,
    seconds: f64,
    deadline_us: u64,
) -> Result<usize, Box<dyn std::error::Error>> {
    println!("commanding {label} for {seconds:.2}s...");
    print_gripper_state(robot, "before")?;
    let deadline = Instant::now() + Duration::from_secs_f64(seconds);
    let tick_deadline = Duration::from_micros(deadline_us);
    let mut ticks = 0;
    let mut next_debug = Instant::now() + Duration::from_millis(250);
    while Instant::now() < deadline {
        {
            let gripper = robot
                .group_mut("grip")
                .and_then(|group| group.as_gripper_mut())
                .ok_or("missing gripper group")?;
            gripper.set_opening(opening, current)?;
        }
        robot.tick(tick_deadline)?;
        ticks += 1;
        if std::env::var_os("CAN_MOTOR_CONTROL_CALIBRATION_DEBUG").is_some()
            && Instant::now() >= next_debug
        {
            println!(
                "  [{label}] tick={ticks} pos={:+.5}",
                gripper_position(robot)?
            );
            next_debug = Instant::now() + Duration::from_millis(250);
        }
        thread::sleep(Duration::from_millis(1));
    }
    print_gripper_state(robot, "after")?;
    Ok(ticks)
}

fn run(args: Args) -> Result<(), Box<dyn std::error::Error>> {
    let motor_type = parse_motor_type(&args.motor_type)
        .ok_or_else(|| format!("unknown --motor-type {}", args.motor_type))?;
    println!("=== Rust 05 gripper normalized opening control ===");
    println!("  - interface          : {}", args.interface);
    println!(
        "  - target             : {}",
        if args.mock {
            "MockCanBus"
        } else {
            &args.interface
        }
    );
    println!(
        "  - send_id / recv_id  : 0x{:02X} / 0x{:02X}",
        args.send_id, args.recv_id
    );
    println!("  - motor_type         : {}", args.motor_type);
    println!("  - default_current    : {}", args.default_current);
    println!(
        "  - lifecycle          : connect -> enable -> automatic opening calibration -> commands"
    );
    println!(
        "  - commands           : {}",
        if args.calibrate_only {
            "calibration only"
        } else {
            "open() -> set_opening(0.5) -> close()"
        }
    );
    println!("  - safety             : clear the gripper jaws before sending commands");
    let _ = io::stdout().flush();

    let transport: Box<dyn CanBus> = if args.mock && args.fd {
        Box::new(MockCanBus::new_fd("vcan_mock"))
    } else if args.mock {
        Box::new(MockCanBus::new("vcan_mock"))
    } else {
        open_native_transport(&args.interface, args.fd)?
    };
    let codec: Box<dyn MotorCodec> = if args.mock {
        Box::new(MockFeedbackCodec::new(args.recv_id))
    } else {
        Box::new(DamiaoCodec::new())
    };
    let mut robot = RobotBuilder::new()
        .add_bus("main", transport, codec)
        .add_gripper_with_opening(
            "grip",
            "main",
            MotorSpec::new("g", motor_type, args.send_id, args.recv_id),
            GripperOpeningSpec::new(args.opening_direction, Some(args.default_current)),
        )
        .build()?;

    robot.connect()?;
    robot.enable()?;

    println!("calibration complete.");
    print_gripper_state(&robot, "after calibration")?;

    if args.calibrate_only {
        println!("calibrate-only requested; disabling and exiting before movement commands.");
        robot.disable()?;
        println!("done.");
        return Ok(());
    }

    let mut ticks = drive_opening_for(
        &mut robot,
        "open()",
        1.0,
        None,
        args.seconds,
        args.deadline_us,
    )?;
    ticks += drive_opening_for(
        &mut robot,
        "set_opening(0.5)",
        0.5,
        args.current,
        args.seconds,
        args.deadline_us,
    )?;
    ticks += drive_opening_for(
        &mut robot,
        "close()",
        0.0,
        None,
        args.seconds,
        args.deadline_us,
    )?;
    robot.disable()?;

    println!("completed {ticks} ticks; done.");
    Ok(())
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(Some(args)) => args,
        Ok(None) => {
            print_help();
            return ExitCode::SUCCESS;
        }
        Err(message) => {
            eprintln!("ERROR: {message}");
            eprintln!("Try --help for usage.");
            return ExitCode::from(2);
        }
    };
    match run(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("ERROR: {error}");
            ExitCode::FAILURE
        }
    }
}
