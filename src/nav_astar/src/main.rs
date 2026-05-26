use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use anyhow::Result;
use rclrs::*;

use geometry_msgs::msg::{
    Point, Pose, PoseStamped, Quaternion, TransformStamped, Twist, TwistStamped, Vector3,
};
use nav_msgs::msg::{OccupancyGrid, Path};
use std_msgs::msg::Header;
use tf2_msgs::msg::TFMessage;

const MAP_FRAME: &str = "map";
const BASE_FRAME: &str = "base_link";
const GOAL_TOLERANCE: f64 = 0.25; // m
const LINEAR_SPEED: f64 = 0.2; // m/s
const ANGULAR_GAIN: f64 = 1.5; // rad/s per rad error
const MAX_ANGULAR: f64 = 1.0; // rad/s clamp
const LOOKAHEAD: f64 = 0.5; // m — pure-pursuit lookahead
const INFLATION_RADIUS: f64 = 0.3; // m — keep path this far from walls
const REPLAN_INTERVAL: f64 = 1.0; // s — recompute path while navigating

// ---------------------------------------------------------------------------
// TF buffer — stores the latest transform for each (parent, child) pair
// ---------------------------------------------------------------------------

#[derive(Default)]
struct TfBuffer {
    transforms: HashMap<(String, String), TransformStamped>,
}

impl TfBuffer {
    fn update(&mut self, msgs: Vec<TransformStamped>) {
        let count_before = self.transforms.len();
        for t in msgs {
            self.transforms.insert(
                (
                    t.header.frame_id.trim_start_matches("/").to_string(),
                    t.child_frame_id.trim_start_matches("/").to_string(),
                ),
                t,
            );
        }
        let count_after = self.transforms.len();
        if count_before != count_after {
            let mut tfs = self
                .transforms
                .iter()
                .map(|v| v.0 .0.clone() + "->" + &v.0 .1)
                .collect::<Vec<_>>();
            tfs.sort();
            eprintln!("available transforms: {tfs:?}");
        }
    }

    fn get(&self, parent: &str, child: &str) -> Option<&TransformStamped> {
        self.transforms.get(&(parent.to_owned(), child.to_owned()))
    }

    // Returns (x, y, yaw) of `target` in `root` frame via BFS over the transform graph.
    fn lookup_pose(&self, root: &str, target: &str) -> Option<(f64, f64, f64)> {
        let mut queue: VecDeque<(String, (f64, f64, f64))> = VecDeque::new();
        let mut visited: HashSet<String> = HashSet::new();
        queue.push_back((root.to_owned(), (0.0, 0.0, 0.0)));
        visited.insert(root.to_owned());

        while let Some((frame, pose)) = queue.pop_front() {
            if frame == target {
                return Some(pose);
            }
            for ((parent, child), t) in &self.transforms {
                if parent != &frame || visited.contains(child) {
                    continue;
                }
                visited.insert(child.clone());
                let (tx, ty, tyaw) = tf_to_xyyaw(t);
                let (px, py, pyaw) = pose;
                let next = (
                    px + tx * pyaw.cos() - ty * pyaw.sin(),
                    py + tx * pyaw.sin() + ty * pyaw.cos(),
                    pyaw + tyaw,
                );
                queue.push_back((child.clone(), next));
            }
        }
        None
    }
}

fn tf_to_xyyaw(t: &TransformStamped) -> (f64, f64, f64) {
    let r = &t.transform.rotation;
    (
        t.transform.translation.x,
        t.transform.translation.y,
        quat_to_yaw(r.x, r.y, r.z, r.w),
    )
}

fn quat_to_yaw(x: f64, y: f64, z: f64, w: f64) -> f64 {
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

// ---------------------------------------------------------------------------
// Shared node state
// ---------------------------------------------------------------------------

struct NavState {
    map: Option<OccupancyGrid>,
    tf: TfBuffer,
    path: Vec<(f64, f64)>,
    waypoint_idx: usize,
    goal: Option<(f64, f64)>,
    last_plan_time: f64,
    cmd_vel_pub: Publisher<TwistStamped>,
    path_pub: Publisher<Path>,
    clock: Clock,
}

// ---------------------------------------------------------------------------
// A* data structures
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, Eq, PartialEq)]
struct AState {
    cost: u32,
    cell: (i32, i32),
}

impl Ord for AState {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for AState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn heuristic(a: (i32, i32), b: (i32, i32)) -> u32 {
    a.0.abs_diff(b.0) + a.1.abs_diff(b.1)
}

fn cell_free(map: &OccupancyGrid, col: i32, row: i32) -> bool {
    if col < 0 || row < 0 {
        return false;
    }
    let (w, h) = (map.info.width as i32, map.info.height as i32);
    if col >= w || row >= h {
        return false;
    }
    map.data[(row * w + col) as usize] <= 0 // 0 = free, -1 = unknown (passable)
}

fn world_to_cell(map: &OccupancyGrid, wx: f64, wy: f64) -> (i32, i32) {
    let res = map.info.resolution as f64;
    let (ox, oy) = (map.info.origin.position.x, map.info.origin.position.y);
    (
        ((wx - ox) / res).floor() as i32,
        ((wy - oy) / res).floor() as i32,
    )
}

fn cell_to_world(map: &OccupancyGrid, col: i32, row: i32) -> (f64, f64) {
    let res = map.info.resolution as f64;
    let (ox, oy) = (map.info.origin.position.x, map.info.origin.position.y);
    (ox + (col as f64 + 0.5) * res, oy + (row as f64 + 0.5) * res)
}

fn inflate_obstacles(map: &OccupancyGrid, radius_m: f64) -> HashSet<(i32, i32)> {
    let res = map.info.resolution as f64;
    let radius_cells = (radius_m / res).ceil() as i32;
    let (w, h) = (map.info.width as i32, map.info.height as i32);
    let mut blocked = HashSet::new();
    for row in 0..h {
        for col in 0..w {
            if map.data[(row * w + col) as usize] > 0 {
                for dr in -radius_cells..=radius_cells {
                    for dc in -radius_cells..=radius_cells {
                        if dr * dr + dc * dc <= radius_cells * radius_cells {
                            let (nc, nr) = (col + dc, row + dr);
                            if nc >= 0 && nc < w && nr >= 0 && nr < h {
                                blocked.insert((nc, nr));
                            }
                        }
                    }
                }
            }
        }
    }
    blocked
}

fn astar(
    map: &OccupancyGrid,
    blocked: &HashSet<(i32, i32)>,
    start: (i32, i32),
    goal: (i32, i32),
) -> Option<Vec<(i32, i32)>> {
    let mut open = BinaryHeap::new();
    let mut g: HashMap<(i32, i32), u32> = HashMap::new();
    let mut came_from: HashMap<(i32, i32), (i32, i32)> = HashMap::new();

    g.insert(start, 0);
    open.push(AState {
        cost: heuristic(start, goal),
        cell: start,
    });

    let neighbors = |col: i32, row: i32| -> [(i32, i32); 8] {
        [
            (col - 1, row),
            (col + 1, row),
            (col, row - 1),
            (col, row + 1),
            (col - 1, row - 1),
            (col + 1, row - 1),
            (col - 1, row + 1),
            (col + 1, row + 1),
        ]
    };

    while let Some(AState { cell, .. }) = open.pop() {
        eprintln!("A* cell: {cell:?}");
        if cell == goal {
            let mut path = vec![goal];
            let mut cur = goal;
            while let Some(&prev) = came_from.get(&cur) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some(path);
        }
        let g_cur = *g.get(&cell).unwrap_or(&u32::MAX);
        for nb in neighbors(cell.0, cell.1) {
            if !cell_free(map, nb.0, nb.1) || blocked.contains(&nb) {
                continue;
            }
            let step = if nb.0 != cell.0 && nb.1 != cell.1 {
                141
            } else {
                100
            };
            let tentative = g_cur.saturating_add(step);
            if tentative < *g.get(&nb).unwrap_or(&u32::MAX) {
                g.insert(nb, tentative);
                came_from.insert(nb, cell);
                open.push(AState {
                    cost: tentative + heuristic(nb, goal),
                    cell: nb,
                });
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Goal callback and replanner
// ---------------------------------------------------------------------------

fn on_goal(state: &mut NavState, goal_msg: PoseStamped) {
    if state.map.is_none() {
        eprintln!("[nav_astar] No map yet, ignoring goal.");
        return;
    }
    state.goal = Some((goal_msg.pose.position.x, goal_msg.pose.position.y));
    replan(state);
}

fn replan(state: &mut NavState) {
    let map = match &state.map {
        Some(m) => m,
        None => return,
    };
    let (rx, ry, _) = match state.tf.lookup_pose(MAP_FRAME, BASE_FRAME) {
        Some(p) => p,
        None => return,
    };
    let (gx, gy) = match state.goal {
        Some(g) => g,
        None => return,
    };
    eprintln!("[nav_astar] Planning from ({rx:.2}, {ry:.2}) to ({gx:.2}, {gy:.2})");

    let inflated = inflate_obstacles(map, INFLATION_RADIUS);
    let cells = match astar(
        map,
        &inflated,
        world_to_cell(map, rx, ry),
        world_to_cell(map, gx, gy),
    ) {
        Some(c) => c,
        None => {
            eprintln!("[nav_astar] A* found no path.");
            return;
        }
    };
    eprintln!("Path found, {} cells", cells.len());

    // Collect world coords while map is still borrowed, then release it.
    let world_path: Vec<(f64, f64)> = cells
        .iter()
        .map(|&(col, row)| cell_to_world(map, col, row))
        .collect();

    state.path = world_path;
    state.waypoint_idx = 0;

    let (sec, nanosec) = state.clock.now().to_sec_nanosec().unwrap();
    state.last_plan_time = sec as f64 + nanosec as f64 * 1e-9;

    let stamp = builtin_interfaces::msg::Time { sec, nanosec };
    let poses = state
        .path
        .iter()
        .map(|&(wx, wy)| PoseStamped {
            header: Header {
                frame_id: MAP_FRAME.to_string(),
                stamp: stamp.clone(),
            },
            pose: Pose {
                position: Point {
                    x: wx,
                    y: wy,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        })
        .collect();

    if let Err(e) = state.path_pub.publish(Path {
        header: Header {
            frame_id: MAP_FRAME.to_string(),
            stamp,
        },
        poses,
    }) {
        eprintln!("[nav_astar] Failed to publish plan: {e}");
    }
}

// ---------------------------------------------------------------------------
// Controller — called on each /tf update
// ---------------------------------------------------------------------------

fn control_step(state: &mut NavState) {
    if state.goal.is_some() {
        let (sec, nanosec) = state.clock.now().to_sec_nanosec().unwrap();
        let now = sec as f64 + nanosec as f64 * 1e-9;
        if now - state.last_plan_time >= REPLAN_INTERVAL {
            replan(state);
        }
    }

    if state.path.is_empty() {
        return;
    }

    let (rx, ry, ryaw) = match state.tf.lookup_pose(MAP_FRAME, BASE_FRAME) {
        Some(p) => p,
        None => return,
    };

    // Prune waypoints the robot has already passed within lookahead distance
    while state.waypoint_idx + 1 < state.path.len() {
        let (wx, wy) = state.path[state.waypoint_idx];
        if ((rx - wx).powi(2) + (ry - wy).powi(2)).sqrt() < LOOKAHEAD {
            state.waypoint_idx += 1;
        } else {
            break;
        }
    }

    let (wx, wy) = state.path[state.waypoint_idx];
    let dist = ((rx - wx).powi(2) + (ry - wy).powi(2)).sqrt();

    if state.waypoint_idx + 1 == state.path.len() && dist < GOAL_TOLERANCE {
        let _ = state.cmd_vel_pub.publish(TwistStamped::default());
        state.path.clear();
        state.goal = None;
        eprintln!("[nav_astar] Goal reached.");
        return;
    }

    let mut heading_err = (wy - ry).atan2(wx - rx) - ryaw;
    while heading_err > std::f64::consts::PI {
        heading_err -= 2.0 * std::f64::consts::PI;
    }
    while heading_err < -std::f64::consts::PI {
        heading_err += 2.0 * std::f64::consts::PI;
    }

    let angular = (ANGULAR_GAIN * heading_err).clamp(-MAX_ANGULAR, MAX_ANGULAR);
    // Reduce forward speed proportionally to heading error; full stop at 90°
    let linear = LINEAR_SPEED * (1.0 - (heading_err.abs() / std::f64::consts::FRAC_PI_2).min(1.0));

    let twist = Twist {
        linear: Vector3 {
            x: linear,
            y: 0.0,
            z: 0.0,
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: angular,
        },
    };
    let (sec, nanosec) = state.clock.now().to_sec_nanosec().unwrap();

    let twist_stamped = TwistStamped {
        header: Header {
            frame_id: BASE_FRAME.to_string(),
            stamp: builtin_interfaces::msg::Time { sec, nanosec },
        },
        twist,
    };
    eprintln!("publishing: {twist_stamped:?}");
    if let Err(e) = state.cmd_vel_pub.publish(twist_stamped) {
        eprintln!("[nav_astar] Failed to publish cmd_vel: {e}");
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("nav_astar")?;

    let cmd_vel_pub = node.create_publisher("/cmd_vel")?;
    let path_pub = node.create_publisher::<Path>("/plan")?;

    let worker = node.create_worker::<NavState>(NavState {
        clock: node.get_clock(),
        map: None,
        tf: TfBuffer::default(),
        path: Vec::new(),
        waypoint_idx: 0,
        goal: None,
        last_plan_time: 0.0,
        cmd_vel_pub,
        path_pub,
    });

    let _map_sub =
        worker.create_subscription("/map", |state: &mut NavState, msg: OccupancyGrid| {
            state.map = Some(msg);
        })?;

    let _goal_sub =
        worker.create_subscription("/goal_pose", |state: &mut NavState, msg: PoseStamped| {
            on_goal(state, msg);
        })?;

    // Dynamic transforms (odom->base_link); also drives the controller
    let _tf_sub = worker.create_subscription("/tf", |state: &mut NavState, msg: TFMessage| {
        state.tf.update(msg.transforms);
        control_step(state);
    })?;

    // Static transforms (map->odom from slam_toolbox, published once)
    let _tf_static_sub = worker.create_subscription(
        {
            let mut opt = SubscriptionOptions::new("/tf_static");
            opt.qos = QoSProfile {
                history: QoSHistoryPolicy::KeepAll,
                durability: QoSDurabilityPolicy::TransientLocal,
                ..QoSProfile::default()
            };
            opt
        },
        |state: &mut NavState, msg: TFMessage| {
            state.tf.update(msg.transforms);
        },
    )?;

    eprintln!("[nav_astar] Node started. Waiting for /map, /tf, /tf_static, and /goal_pose.");
    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
