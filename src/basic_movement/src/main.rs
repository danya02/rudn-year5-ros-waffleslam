use anyhow::Result;
use rclrs::{
    Context, CreateBasicExecutor, PublisherOptions, RclrsErrorFilter, SpinOptions,
    SubscriptionOptions,
};
use std::sync::Arc;

const FRONT_ANGLE: f32 = 30.0; // degrees each side of forward to check
const STOP_DISTANCE: f32 = 0.5; // m — start turning when obstacle closer than this
const LINEAR_SPEED: f64 = 0.7; // m/s
const TURN_SPEED: f64 = 0.8; // rad/s

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("obstacle_avoider")?;

    let publisher = node.create_publisher(PublisherOptions::new("cmd_vel"))?;

    let clock = node.get_clock();
    let _subscription = {
        let publisher = Arc::clone(&publisher);
        node.create_subscription(
            SubscriptionOptions::new("scan"),
            move |msg: sensor_msgs::msg::LaserScan| {
                let front_rad = FRONT_ANGLE.to_radians();
                let mut min_dist = f32::INFINITY;

                for (i, &r) in msg.ranges.iter().enumerate() {
                    let angle = msg.angle_min + i as f32 * msg.angle_increment;
                    if angle.abs() <= front_rad && msg.range_min < r && r < msg.range_max {
                        if r < min_dist {
                            min_dist = r;
                        }
                    }
                }

                let mut cmd = geometry_msgs::msg::TwistStamped::default();
                if min_dist < STOP_DISTANCE {
                    eprintln!("{min_dist} below stop distance, turning");
                    cmd.twist.angular.z = TURN_SPEED;
                } else {
                    eprintln!("{min_dist} above stop distance, driving");
                    cmd.twist.linear.x = LINEAR_SPEED;
                }

                let (sec, nanosec) = clock.now().to_sec_nanosec().unwrap();
                cmd.header.stamp = builtin_interfaces::msg::Time { sec, nanosec };

                if let Err(e) = publisher.publish(cmd) {
                    eprintln!("Failed to publish cmd_vel: {e}");
                }
            },
        )?
    };

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
