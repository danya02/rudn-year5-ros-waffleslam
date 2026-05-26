SHELL    := /bin/bash
WS_DIR   := $(shell pwd)
ROS      := source /opt/ros/jazzy/setup.bash
INSTALL  := source $(WS_DIR)/install/setup.bash
CARGO    := PATH=$$PATH:$$HOME/.cargo/bin

# RViz config — save your layout here then it will be reloaded automatically
RVIZ_CONFIG ?= $(WS_DIR)/rviz/main.rviz

.PHONY: build build-main build-astar \
        launch launch-slam launch-nav2 launch-astar \
        square avoid teleop \
        basic-movement square-movement \
        rviz save-map \
        astar-goal astar-path \
        nodes topics

# ── Build ──────────────────────────────────────────────────────────────────

build:
	$(ROS) && $(CARGO) colcon build

build-main:
	$(ROS) && $(CARGO) colcon build --packages-select main

build-astar:
	$(ROS) && $(CARGO) colcon build --packages-select nav_astar

# ── Launch ─────────────────────────────────────────────────────────────────

# Full stack: Gazebo + SLAM + Nav2 + A* node
launch: build
	$(ROS) && $(INSTALL) && ros2 launch main main.py

# Gazebo + TurtleBot3 + slam_toolbox (task1 sim, task2 mapping stage)
launch-slam: build
	$(ROS) && $(INSTALL) && ros2 launch main slam.py

# Nav2 stack only — run alongside launch-slam (or a map_server)
launch-nav2: build
	$(ROS) && $(INSTALL) && ros2 launch main nav2.py

# Custom A* planner only
launch-astar: build
	$(ROS) && $(INSTALL) && ros2 launch main astar.py

# ── Python nodes ───────────────────────────────────────────────────────────

# Keyboard teleoperation (useful while SLAM is building a map)
teleop:
	$(ROS) && TURTLEBOT3_MODEL=waffle ros2 run turtlebot3_teleop teleop_keyboard

# ── Rust movement nodes ────────────────────────────────────────────────────

# Basic open-loop movement on /cmd_vel
basic-movement:
	$(ROS) && $(INSTALL) && ros2 run basic_movement basic_movement

# Drive a square via the Rust node
square-movement:
	$(ROS) && $(INSTALL) && ros2 run square_movement square_movement

# ── Direct velocity injection (for debugging) ──────────────────────────────

# Publish TwistStamped directly to /cmd_vel — bypasses all of Nav2.
# If this moves the robot the bridge+Gazebo side is fine.
# Ctrl-C to stop (sends a zero-velocity message first).
cmd-vel:
	$(ROS) && $(INSTALL) && \
		ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
		"{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}" \
		--rate 10

# ── Map ────────────────────────────────────────────────────────────────────

# Save the SLAM map (run while slam_toolbox is active)
save-map:
	$(ROS) && $(INSTALL) && \
		ros2 service call /slam_toolbox/save_map \
		slam_toolbox/srv/SaveMap "{name: {data: map1}}"

# ── A* planner ─────────────────────────────────────────────────────────────

# Send a goal to the A* planner:  make astar-goal X=1.0 Y=1.5
X ?= 1.0
Y ?= 1.0
astar-goal:
	$(ROS) && $(INSTALL) && \
		ros2 topic pub --once /astar_goal geometry_msgs/msg/PoseStamped \
		"{header: {frame_id: 'map'}, \
		  pose: {position: {x: $(X), y: $(Y), z: 0.0}, \
		         orientation: {w: 1.0}}}"

# Stream the computed A* path
astar-path:
	$(ROS) && $(INSTALL) && ros2 topic echo /astar_path

# ── Visualisation ──────────────────────────────────────────────────────────

# Open RViz; loads saved config if it exists, otherwise starts blank
rviz:
	$(ROS) && $(INSTALL) && \
		if [ -f "$(RVIZ_CONFIG)" ]; then \
			rviz2 -d $(RVIZ_CONFIG); \
		else \
			rviz2; \
		fi

# ── Introspection ──────────────────────────────────────────────────────────

nodes:
	$(ROS) && $(INSTALL) && ros2 node list

topics:
	$(ROS) && $(INSTALL) && ros2 topic list
