use anyhow::Result;
use builtin_interfaces;
use geometry_msgs::msg::{Quaternion, Twist, TwistStamped, Vector3};
use nav_msgs::msg::Odometry;
use rclrs::{
    Clock, Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, Time, TimerOptions,
};
use std::f64::consts::{FRAC_PI_2, PI};
use std::time::Duration;

const SPEED: f64 = 0.3; // m/s
const SIDE_LEN: f64 = 2.0; // m, full side length
const CORNER_RADIUS: f64 = 0.3; // m

// Stop the turn once we are within this many radians of π/2 (≈1°).
const TURN_TOLERANCE: f64 = 0.02;

#[derive(Debug)]
enum Phase {
    Straight,
    Turning,
    Done,
}

struct DriveState {
    cmd_vel: rclrs::Publisher<TwistStamped>,
    clock: Clock,
    phase: Phase,
    leg: u32,
    phase_start: Time,
    straight_dur: Duration,
    turn_rate: f64,
    current_heading: Option<f64>, // radians, from /odom
    turn_start_heading: f64,
}

fn yaw_from_quaternion(q: &Quaternion) -> f64 {
    f64::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
}

// Signed angular difference current − start, normalized to (−π, π].
fn angle_turned(current: f64, start: f64) -> f64 {
    let d = (current - start).rem_euclid(2.0 * PI);
    if d > PI {
        d - 2.0 * PI
    } else {
        d
    }
}

fn elapsed_since(clock: &Clock, start: &Time) -> Duration {
    Duration::from_nanos((clock.now().nsec - start.nsec).max(0) as u64)
}

impl DriveState {
    fn new(cmd_vel: rclrs::Publisher<TwistStamped>, clock: Clock) -> Self {
        let turn_rate = SPEED / CORNER_RADIUS;
        let now = clock.now();
        Self {
            cmd_vel,
            clock,
            phase: Phase::Straight,
            leg: 0,
            phase_start: now,
            straight_dur: Duration::from_secs_f64((SIDE_LEN - 2.0 * CORNER_RADIUS) / SPEED),
            turn_rate,
            current_heading: None,
            turn_start_heading: 0.0,
        }
    }

    fn publish(&self, linear_x: f64, angular_z: f64) {
        let (sec, nanosec) = self.clock.now().to_sec_nanosec().unwrap();
        eprintln!("phase={:?} twist: x={linear_x} z={angular_z}", self.phase);
        let _ = self.cmd_vel.publish(TwistStamped {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time { sec, nanosec },
                frame_id: "base_link".to_string(),
            },
            twist: Twist {
                linear: Vector3 {
                    x: linear_x,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: angular_z,
                },
            },
        });
    }

    fn tick(&mut self) {
        match self.phase {
            Phase::Done => self.publish(0.0, 0.0),

            Phase::Straight => {
                if elapsed_since(&self.clock, &self.phase_start) < self.straight_dur {
                    self.publish(SPEED, 0.0);
                } else {
                    self.turn_start_heading = self.current_heading.unwrap_or(0.0);
                    self.phase = Phase::Turning;
                    self.phase_start = self.clock.now();
                    self.publish(SPEED, self.turn_rate);
                }
            }

            Phase::Turning => {
                let done = match self.current_heading {
                    Some(h) => {
                        angle_turned(h, self.turn_start_heading) >= FRAC_PI_2 - TURN_TOLERANCE
                    }
                    None => {
                        let fallback = Duration::from_secs_f64(FRAC_PI_2 / self.turn_rate);
                        elapsed_since(&self.clock, &self.phase_start) >= fallback
                    }
                };

                if done {
                    self.leg += 1;
                    if self.leg >= 4 {
                        self.phase = Phase::Done;
                        eprintln!("[square_movement] Rounded square complete.");
                        self.publish(0.0, 0.0);
                    } else {
                        self.phase = Phase::Straight;
                        self.phase_start = self.clock.now();
                        self.publish(SPEED, 0.0);
                    }
                } else {
                    self.publish(SPEED, self.turn_rate);
                }
            }
        }
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("square_movement_node")?;

    let cmd_vel = node.create_publisher::<TwistStamped>("/cmd_vel")?;
    let worker = node.create_worker::<DriveState>(DriveState::new(cmd_vel, node.get_clock()));

    let _odom_sub =
        worker.create_subscription("/odom", |state: &mut DriveState, msg: Odometry| {
            state.current_heading = Some(yaw_from_quaternion(&msg.pose.pose.orientation));
        })?;

    let _timer = worker.create_timer_repeating(
        TimerOptions::new(Duration::from_millis(50)),
        |state: &mut DriveState| {
            state.tick();
        },
    )?;

    eprintln!(
        "[square_movement] Starting rounded square \
         (side={SIDE_LEN}m, corner_radius={CORNER_RADIUS}m, speed={SPEED}m/s)"
    );
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
