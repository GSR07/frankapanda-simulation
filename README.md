# Franka Panda Pick-and-Place Project — Complete Beginner's Guide

---

## 1. PROJECT OVERVIEW — What Are We Building?

### The Big Picture

This project makes a **Franka Emika Panda** robotic arm (a 7-joint industrial arm with a 2-finger gripper) **see colored objects through a camera, decide which one to pick, move its arm to grab it, lift it, and drop it at a different location** — all inside a simulation.

### Real-World Analogy

Imagine a factory conveyor belt with Red, Green, and Blue blocks. You tell the robot: "Pick the Blue one." The robot looks through its camera, finds the blue block, calculates where it is in 3D space, plans a collision-free path to reach it, grabs it, lifts it, moves to a drop zone, and releases it.

### What Technologies Are Involved?

| Technology | Role in This Project |
|---|---|
| **ROS 2 (Humble)** | The "operating system" for the robot — connects all nodes |
| **Ignition Gazebo (Fortress)** | The physics simulator — renders the world, applies gravity, handles collisions |
| **URDF/Xacro** | Describes the robot's physical structure (links, joints, meshes) |
| **ros2_control** | The bridge between ROS 2 and the simulated joint motors |
| **MoveIt 2** | Motion planning — finds collision-free paths from point A to point B |
| **pymoveit2** | A Python wrapper around MoveIt 2 so you can plan/execute motion from Python |
| **OpenCV** | Computer vision — detects colored objects in the camera image |
| **TF2** | Coordinate transform library — converts camera coordinates to robot coordinates |
| **RViz 2** | 3D visualization tool — shows the robot, planned paths, and planning scene |

---

## 2. COMPLETE REPOSITORY STRUCTURE — What Each Folder/File Does

```
pandaws/src/
├── panda_description/      ← Robot's physical body definition (URDF, meshes, Gazebo world)
│   ├── urdf/
│   │   ├── panda.urdf.xacro        ← Main robot description file
│   │   └── arm.xacro               ← Arm-specific Xacro (referenced by MoveIt setup)
│   ├── meshes/                      ← 3D mesh files (.dae/.stl) for visual/collision
│   ├── worlds/                      ← Gazebo world files (.sdf) with tables, blocks, camera
│   └── launch/
│       └── gazebo.launch.py         ← Launches Ignition Gazebo with the robot + world
│
├── panda_controller/       ← Joint controllers — tells ros2_control HOW to move each joint
│   ├── config/
│   │   └── controllers.yaml         ← Defines arm_controller, gripper_controller, joint_state_broadcaster
│   ├── launch/
│   │   ├── controller.launch.py     ← Spawns robot_state_publisher + controller_manager + controllers
│   │   └── slider_controller.launch.py ← Debugging tool: manual sliders to move each joint
│   ├── panda_controller/
│   │   └── slider_controller.py     ← Python node that bridges slider GUI → joint trajectory commands
│   ├── CMakeLists.txt
│   └── package.xml
│
├── panda_moveit/           ← MoveIt 2 configuration — motion planning
│   ├── config/
│   │   ├── panda.srdf               ← Semantic Robot Description: groups, named poses, collision pairs
│   │   ├── kinematics.yaml          ← Kinematics solver config (KDL plugin)
│   │   ├── joint_limits.yaml        ← Velocity/acceleration limits per joint
│   │   ├── moveit_controllers.yaml  ← Tells MoveIt which controllers to use for execution
│   │   ├── initial_positions.yaml   ← Default starting joint positions
│   │   └── .setup_assistant         ← MoveIt Setup Assistant metadata
│   ├── launch/
│   │   └── movit.launch.py          ← Launches move_group node + RViz with MoveIt plugin
│   ├── rviz/
│   │   └── moveit.rviz              ← Preconfigured RViz layout for MoveIt
│   ├── CMakeLists.txt
│   └── package.xml
│
├── panda_vision/           ← Computer vision — detects colored objects
│   ├── panda_vision/
│   │   ├── __init__.py
│   │   └── color_detector.py        ← THE VISION NODE: camera → HSV → contour → 3D coords
│   ├── setup.py                     ← Python package setup
│   ├── setup.cfg
│   └── package.xml
│
├── pymoveit2/              ← Python MoveIt 2 interface library + pick-and-place logic
│   ├── pymoveit2/
│   │   ├── __init__.py
│   │   ├── moveit2.py               ← Core MoveIt2 class: plan, execute, pose goals, joint goals
│   │   ├── moveit2_gripper.py       ← Gripper control via MoveIt (JointTrajectoryController)
│   │   ├── moveit2_servo.py         ← Real-time servo control interface
│   │   ├── gripper_command.py       ← Gripper control via GripperCommand action
│   │   ├── gripper_interface.py     ← Unified gripper interface (auto-detects which backend to use)
│   │   ├── robots/
│   │   │   └── panda.py             ← Panda-specific constants: joint names, group names, gripper positions
│   │   └── utils.py                 ← Utility functions
│   ├── examples/
│   │   ├── pick_and_place.py        ← ★ THE MAIN SCRIPT: full pick-and-place sequence
│   │   ├── ex_joint_goal.py         ← Move arm to a joint configuration
│   │   ├── ex_pose_goal.py          ← Move arm to a Cartesian pose
│   │   ├── ex_gripper.py            ← Open/close/toggle gripper
│   │   ├── ex_collision_primitive.py ← Add simple collision objects to planning scene
│   │   ├── ex_collision_mesh.py     ← Add mesh collision objects
│   │   ├── ex_allow_collisions.py   ← Enable/disable collision checking for objects
│   │   ├── ex_clear_planning_scene.py ← Clear all collision objects
│   │   ├── ex_orientation_path_constraint.py ← Move with orientation constraints
│   │   ├── ex_fk.py                 ← Forward kinematics example
│   │   ├── ex_ik.py                 ← Inverse kinematics example
│   │   └── ex_servo.py              ← Real-time servo example
│   ├── CMakeLists.txt
│   └── package.xml
│
├── panda_bringup/          ← Master launch package — launches EVERYTHING together
│   ├── launch/
│   │   └── pick_and_place.launch.py ← THE ONE COMMAND: Gazebo + Controllers + MoveIt + Vision + Pick&Place
│   └── package.xml
│
├── .gitignore
└── README.md
```

---

## 3. KEY CONCEPTS EXPLAINED (Beginner-Friendly)

### 3.1 What is a URDF / Xacro?

**URDF** (Unified Robot Description Format) is an XML file that describes a robot's body:
- **Links**: The rigid physical parts (like bones in a human body). Each link has visual geometry (what it looks like), collision geometry (simplified shape for physics), and inertial properties (mass, center of mass).
- **Joints**: The connections between links that allow movement. The Panda has 7 revolute (rotating) joints in the arm and 2 prismatic (sliding) joints in the gripper fingers.

**Xacro** is a macro language that extends URDF. Instead of writing 1000 lines of XML, you write macros, use variables, and use conditionals. The `.xacro` file is processed into a standard URDF at build time.

Example: `panda.urdf.xacro` is the main file. It likely includes the arm description plus ros2_control hardware interfaces and sensor definitions, and accepts parameters like `is_sim` and `is_ignition` to switch between real hardware and simulation.

### 3.2 What is ros2_control?

Think of `ros2_control` as the **electrical wiring** between your software and the motors. It provides:

1. **Hardware Interfaces** — Abstract the real/simulated motors. In simulation, `ign_ros2_control` talks to Ignition Gazebo joints.
2. **Controllers** — Algorithms that take commands and send them to hardware:
   - `joint_trajectory_controller` — Takes a trajectory (sequence of joint positions over time) and follows it
   - `joint_state_broadcaster` — Reads current joint positions and publishes them as `/joint_states`

Your `controllers.yaml` configures these:
```yaml
# The arm controller manages all 7 arm joints
arm_controller:
  ros__parameters:
    joints:
      - panda_joint1 through panda_joint7
    command_interfaces: [position]    # We send position commands
    state_interfaces: [position]      # We read position feedback
    open_loop_control: true           # No feedback loop (simpler for simulation)

# The gripper controller manages the 2 finger joints
gripper_controller:
  ros__parameters:
    joints:
      - panda_finger_joint1
      - panda_finger_joint2
    command_interfaces: [position]
    state_interfaces: [position]
```

### 3.3 What is MoveIt 2?

MoveIt 2 is the **brain** that plans how the arm should move. When you say "move the end-effector to position [0.5, 0.0, 0.3]", MoveIt:

1. Takes the target pose
2. Uses Inverse Kinematics (IK) to calculate what joint angles achieve that pose
3. Uses a motion planner (like RRT — Rapidly-exploring Random Trees) to find a path that avoids collisions
4. Generates a smooth trajectory (sequence of joint positions over time)
5. Sends the trajectory to the controller for execution

**Key configuration files:**

- **SRDF** (`panda.srdf`): The Semantic Robot Description Format. It defines:
  - **Planning groups**: "arm" (link0→link7) and "gripper" (hand + fingers)
  - **Named poses**: "home", "open", "close" — predefined joint configurations
  - **Collision matrix**: Which link pairs to skip collision checking (e.g., adjacent links never collide)
  - **Virtual joint**: Connects the robot base to the world frame

- **kinematics.yaml**: Configures the KDL (Kinematics and Dynamics Library) solver for IK calculations

- **joint_limits.yaml**: Overrides URDF velocity/acceleration limits. Your project uses scaling factors of 0.1 (10% of max speed) for safety

- **moveit_controllers.yaml**: Tells MoveIt which ROS 2 controllers to use:
  ```yaml
  arm_controller:
    action_ns: follow_joint_trajectory   # The action name
    type: FollowJointTrajectory          # Action type
    joints: [panda_joint1 ... panda_joint7]
  ```

### 3.4 What is TF2 (Transform Library)?

TF2 maintains a **tree of coordinate frames**. Every link in the robot has its own coordinate frame. When the camera sees an object at pixel (320, 160), the vision node needs to figure out where that is relative to the robot's base. TF2 provides the chain of transforms:

```
world → panda_link0 → panda_link1 → ... → panda_link7 → panda_hand → camera_link
```

The vision node uses `tf_buffer.lookup_transform("panda_link0", "camera_link", ...)` to get the 4x4 transformation matrix between the camera and the robot base, then multiplies the 3D point by this matrix.

### 3.5 What are ROS 2 Topics, Nodes, and Actions?

- **Node**: A single executable process. Each node does one thing. Examples: `color_detector`, `pick_and_place`, `move_group`, `controller_manager`
- **Topic**: A named channel for one-way data. Nodes publish to topics and subscribe to them. Example: the camera publishes images to `/camera/image_raw`, the vision node subscribes to it
- **Action**: A request-response pattern with feedback for long-running tasks. Example: MoveIt sends a trajectory to `follow_joint_trajectory` action, gets progress feedback, and finally gets a result (success/failure)

---

## 4. DETAILED CODE WALKTHROUGH — Every File Explained

---

### 4.1 `panda_vision/color_detector.py` — The Vision Node

**Purpose**: Detects red, green, and blue objects in the camera image and publishes their 3D coordinates in the robot's base frame.

**How it works step by step:**

```
Camera Image → Convert to HSV → Color Mask → Find Contours →
Calculate Pixel Center → Convert Pixel to 3D Point (Camera Frame) →
Look up TF Transform → Convert to Robot Base Frame → Publish Coordinates
```

**Detailed code explanation:**

```python
class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        # This creates a ROS 2 node named "color_detector"

        # SUBSCRIBER: Listens to camera images
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        # Image = message type (sensor_msgs/Image)
        # '/camera/image_raw' = topic name published by camera in Gazebo
        # self.image_callback = function to call when image arrives
        # 10 = queue size (buffer up to 10 messages)

        # PUBLISHER: Sends detected object coordinates
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)
        # Publishes strings like "B,0.450,0.120,0.850" (color,x,y,z)

        # OpenCV Bridge: Converts between ROS Image messages and OpenCV arrays
        self.bridge = CvBridge()

        # TF2: For looking up coordinate transforms between frames
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsic parameters (from the simulated camera's SDF)
        # These define the camera's field of view and distortion
        self.fx = 585.0  # Focal length X (pixels)
        self.fy = 588.0  # Focal length Y (pixels)
        self.cx = 320.0  # Principal point X (image center)
        self.cy = 160.0  # Principal point Y (image center)
```

**The image callback — where the magic happens:**

```python
def image_callback(self, msg):
    # Step 1: Convert ROS Image → OpenCV BGR numpy array
    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Step 2: Convert BGR → HSV (Hue-Saturation-Value)
    # HSV is better for color detection because hue is independent of brightness
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Step 3: Define color ranges in HSV space
    color_ranges = {
        "R": [(0, 120, 70), (10, 255, 255)],     # Red hue range
        "G": [(55, 200, 200), (60, 255, 255)],    # Green hue range
        "B": [(90, 200, 200), (128, 255, 255)]    # Blue hue range
    }

    # Step 4: For each color, create a binary mask
    for color_id, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        # mask = white where color matches, black elsewhere

        # Step 5: Clean up noise
        mask = cv2.erode(mask, None, iterations=2)   # Remove small specks
        mask = cv2.dilate(mask, None, iterations=2)   # Fill small holes

        # Step 6: Find contours (outlines of detected regions)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 1:  # Filter tiny detections
                # Step 7: Get bounding box and center pixel
                x, y, w, h = cv2.boundingRect(cnt)
                cx_pix, cy_pix = x + w // 2, y + h // 2

                # Step 8: Convert pixel coordinates to 3D camera coordinates
                # Using the pinhole camera model:
                #   X = (pixel_x - cx) * Z / fx
                #   Y = (pixel_y - cy) * Z / fy
                Z = 0.1  # Assumed depth (distance to object)
                Y = (cx_pix - self.cx) * Z / self.fx * -10
                X = (cy_pix - self.cy) * Z / self.fy
                # Note: The *-10 and axis swapping is to account for
                # camera vs robot coordinate conventions

                # Step 9: Look up transform from camera_link to panda_link0
                t = self.tf_buffer.lookup_transform(
                    "panda_link0",   # Target frame (robot base)
                    "camera_link",   # Source frame (camera)
                    rclpy.time.Time())  # Latest available

                # Step 10: Build 4x4 transformation matrix
                T = tf_transformations.quaternion_matrix(rot)
                T[:3, 3] = trans

                # Step 11: Transform point from camera frame to base frame
                pt_cam = np.array([X, Y, Z, 1.0])
                pt_base = T @ pt_cam  # Matrix multiplication!

                # Step 12: Apply manual calibration offsets
                if color_id == "B":
                    pt_base[1] -= 0.0215  # Blue needs slight Y adjustment
                elif color_id == "G":
                    pt_base[1] += 0.01    # Green needs slight Y adjustment

                # Step 13: Publish as "COLOR,X,Y,Z" string
                msg_str = f"{color_id},{pt_base[0]:.3f},{pt_base[1]:.3f},{pt_base[2]:.3f}"
                self.coords_pub.publish(String(data=msg_str))
```

**Why do we need calibration offsets?** In simulation, there are small discrepancies between the theoretical camera model and actual rendering. The offsets (`-0.0215` for Blue, `+0.01` for Green) are empirical corrections found through testing to make the picking more accurate.

---

### 4.2 `pymoveit2/examples/pick_and_place.py` — The Main Pick-and-Place Node

**Purpose**: Listens for a target color's 3D coordinates, then executes a full pick-and-place motion sequence.

**The motion sequence (11 steps):**

```
1. Move to START position (all joints zero except wrist)
2. Wait for target color coordinates from vision node
3. Lock coordinates (ignore further updates)
4. Move to HOME position (arm in ready pose)
5. Move ABOVE the object (Cartesian motion)
6. Open gripper
7. Move DOWN to the object (approach)
8. Close gripper (grab)
9. Move to HOME position (lift)
10. Move to DROP position (rotate arm to drop zone)
11. Open gripper (release)
12. Close gripper (reset)
13. Return to START position
```

**Detailed code explanation:**

```python
class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # ROS 2 Parameter: which color to pick (R, G, or B)
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        # Flags to ensure we only pick once
        self.already_moved = False
        self.target_coords = None

        # ReentrantCallbackGroup: Allows multiple callbacks to run simultaneously
        # This is CRITICAL because MoveIt needs to process feedback while we wait
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface for the arm
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),         # ["panda_joint1", ..., "panda_joint7"]
            base_link_name=panda.base_link_name(),   # "panda_link0"
            end_effector_name=panda.end_effector_name(), # "panda_hand"
            group_name=panda.MOVE_GROUP_ARM,         # "arm"
            callback_group=self.callback_group,
        )

        # Slow motion for safety: 10% of max speed
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Create Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,   # [0.04, 0.04]
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS, # [0.0, 0.0]
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscribe to color coordinates from vision node
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10)
```

**The predefined joint configurations:**

```python
# START: All zeros except last joint rotated -125 degrees
# This puts the arm in a compact upright position
self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]

# HOME: A "ready to reach" pose
# Joint 4 at -90°, Joint 6 at 92°, Joint 7 at 50°
self.home_joints = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0,
                    math.radians(92.0), math.radians(50.0)]

# DROP: The arm swings to the opposite side to release the object
# Joint 1 at -155° (swung far left), various other angles
self.drop_joints = [math.radians(-155.0), math.radians(30.0),
                    math.radians(-20.0), math.radians(-124.0),
                    math.radians(44.0), math.radians(163.0),
                    math.radians(7.0)]
```

**The coordinate callback — triggered when vision detects the target:**

```python
def coords_callback(self, msg):
    if self.already_moved:
        return  # Already picked — ignore further messages

    color_id, x, y, z = msg.data.split(",")
    if color_id.strip().upper() == self.target_color:
        # LOCK the coordinates immediately
        self.target_coords = [float(x), float(y), float(z)]
        self.already_moved = True

        # Calculate pick position (offset Z by -0.60 for approach height)
        pick_position = [self.target_coords[0],
                         self.target_coords[1],
                         self.target_coords[2] - 0.60]

        # Orientation: gripper pointing straight down
        # Quaternion [0, 1, 0, 0] = 180° rotation around Y axis
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]

        # Execute the 11-step sequence:
        # 1. Go to home
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Move above object (Cartesian)
        self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 3. Open gripper
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 4. Move down to object
        approach_position = [pick_position[0], pick_position[1],
                             pick_position[2] - 0.30]
        self.moveit2.move_to_pose(position=approach_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 5. Close gripper (GRAB)
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 6. Move to home (LIFT)
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 7. Move to drop zone
        self.moveit2.move_to_configuration(self.drop_joints)
        self.moveit2.wait_until_executed()

        # 8. Open gripper (RELEASE)
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 9. Close gripper (reset)
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 10. Return to start
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

        rclpy.shutdown()  # Done!
```

**Why use `move_to_configuration` vs `move_to_pose`?**
- `move_to_configuration(joint_list)` — Move by specifying exact joint angles. Guaranteed to reach the exact pose. Used for known safe positions (home, start, drop).
- `move_to_pose(position, quat_xyzw)` — Move by specifying where the end-effector should be in 3D space. MoveIt solves inverse kinematics. Used for reaching dynamically detected objects.

---

### 4.3 `pymoveit2/robots/panda.py` — Robot Constants

**Purpose**: A simple data file that stores Panda-specific constants so they don't get hardcoded everywhere.

```python
MOVE_GROUP_ARM: str = "arm"             # Planning group name for arm
MOVE_GROUP_GRIPPER: str = "gripper"     # Planning group name for gripper

# Gripper finger positions (in meters)
OPEN_GRIPPER_JOINT_POSITIONS = [0.04, 0.04]    # Each finger opens 4cm
CLOSED_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]    # Fingers fully closed

def joint_names(prefix="panda_"):
    return [prefix + "joint1", ..., prefix + "joint7"]

def base_link_name(prefix="panda_"):
    return prefix + "link0"

def end_effector_name(prefix="panda_"):
    return prefix + "hand"

def gripper_joint_names(prefix="panda_"):
    return [prefix + "finger_joint1", prefix + "finger_joint2"]
```

---

### 4.4 `panda_controller/slider_controller.py` — Manual Joint Control

**Purpose**: A debugging tool. It bridges the `joint_state_publisher_gui` (slider UI) to the actual controllers, letting you manually drag sliders to move each joint.

```python
class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        # Publishers for arm and gripper trajectory topics
        self.arm_pub_ = self.create_publisher(
            JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(
            JointTrajectory, "gripper_controller/joint_trajectory", 10)
        # Subscriber: reads from the remapped joint_commands topic
        self.sub_ = self.create_subscription(
            JointState, "joint_commands", self.sliderCallback, 10)

    def sliderCallback(self, msg):
        # Take slider values and send them as trajectory commands
        arm_goal = JointTrajectoryPoint()
        arm_goal.positions = msg.position[:7]   # First 7 = arm joints
        gripper_goal = JointTrajectoryPoint()
        gripper_goal.positions = [msg.position[7]]  # 8th = finger

        # Wrap in JointTrajectory messages and publish
        arm_controller = JointTrajectory()
        arm_controller.joint_names = ["panda_joint1", ..., "panda_joint7"]
        arm_controller.points.append(arm_goal)
        self.arm_pub_.publish(arm_controller)

        gripper_controller = JointTrajectory()
        gripper_controller.joint_names = ["panda_finger_joint1"]
        gripper_controller.points.append(gripper_goal)
        self.gripper_pub_.publish(gripper_controller)
```

---

### 4.5 LAUNCH FILES — How Everything Starts

#### `panda_bringup/launch/pick_and_place.launch.py` — The Master Launcher

This is the **single command** that starts the entire project. It chains together 5 sub-launches/nodes:

```python
def generate_launch_description():
    # 1. GAZEBO: Start Ignition Gazebo with the robot in the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("panda_description"),
                         "launch", "gazebo.launch.py")))

    # 2. CONTROLLERS: Start ros2_control + spawn arm/gripper controllers
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("panda_controller"),
                         "launch", "controller.launch.py")),
        launch_arguments={"is_sim": "True"}.items())

    # 3. MOVEIT: Start move_group (planning) + RViz (visualization)
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("panda_moveit"),
                         "launch", "movit.launch.py")),
        launch_arguments={"is_sim": "True"}.items())

    # 4. VISION: Start color detection node
    vision_node = Node(
        package="panda_vision",
        executable="color_detector",
        name="color_detector",
        output="screen")

    # 5. PICK AND PLACE: Start the pick-and-place behavior
    color_picker_node = Node(
        package="pymoveit2",
        executable="pick_and_place.py",
        name="pick_and_place",
        output="screen",
        parameters=[{"target_color": "B"}])  # Change to "R" or "G" as needed

    return LaunchDescription([gazebo, controller, moveit, vision_node, color_picker_node])
```

#### `panda_controller/launch/controller.launch.py` — Controller Setup

```python
def generate_launch_description():
    # 1. Process Xacro → URDF (with simulation flags)
    robot_description = ParameterValue(
        Command(["xacro ", path_to_xacro,
                 " is_sim:=True", " is_ignition:=True"]),
        value_type=str)

    # 2. Robot State Publisher: Publishes TF transforms based on URDF + joint states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    # 3. Controller Manager: The central hub for all controllers
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml])

    # 4. Spawners: Activate each controller
    joint_state_broadcaster_spawner = Node(...)   # Publishes /joint_states
    arm_controller_spawner = Node(...)            # Arm trajectory controller
    gripper_controller_spawner = Node(...)        # Gripper trajectory controller
```

#### `panda_moveit/launch/movit.launch.py` — MoveIt Setup

```python
def generate_launch_description():
    # Build MoveIt configuration from all config files
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit")
        .robot_description(file_path=path_to_xacro, mappings={"is_ignition": "true"})
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # move_group: The MoveIt planning server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}])

    # RViz with MoveIt plugin
    rviz_node = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits])
```

---

### 4.6 Configuration Files Explained

#### `controllers.yaml` — What controllers exist and what joints they control

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100    # Run controller loop at 100 Hz

    # Declare controller types
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints: [panda_joint1, ..., panda_joint7]
    command_interfaces: [position]   # Send position commands
    state_interfaces: [position]     # Read position feedback
    open_loop_control: true          # No PID feedback (sim is ideal)

gripper_controller:
  ros__parameters:
    joints: [panda_finger_joint1, panda_finger_joint2]
    command_interfaces: [position]
    state_interfaces: [position]
    open_loop_control: true
```

#### `panda.srdf` — The Semantic Robot Description

```xml
<robot name="panda">
    <!-- PLANNING GROUPS: Which joints move together -->
    <group name="arm">
        <chain base_link="panda_link0" tip_link="panda_link7"/>
    </group>
    <group name="gripper">
        <link name="panda_hand"/>
        <link name="panda_leftfinger"/>
        <link name="panda_rightfinger"/>
        <joint name="panda_finger_joint1"/>
        <joint name="panda_finger_joint2"/>
    </group>

    <!-- NAMED POSES: Shortcut configurations -->
    <group_state name="home" group="arm">
        <!-- Joint 4 at -90°, Joint 6 at ~92°, Joint 7 at ~50° -->
    </group_state>
    <group_state name="open" group="gripper">
        <!-- Fingers at 0.04m (4cm apart) -->
    </group_state>
    <group_state name="close" group="gripper">
        <!-- Fingers at 0.0m (fully closed) -->
    </group_state>

    <!-- VIRTUAL JOINT: Connects robot to world (fixed = robot doesn't move) -->
    <virtual_joint name="virtual_joint" type="fixed"
                   parent_frame="world" child_link="panda_link0"/>

    <!-- COLLISION MATRIX: Skip collision checks between link pairs
         that can never collide (saves computation during planning) -->
    <disable_collisions link1="panda_link0" link2="panda_link1" reason="Adjacent"/>
    <!-- ... many more pairs ... -->
</robot>
```

#### `moveit_controllers.yaml` — Tells MoveIt how to execute plans

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names: [arm_controller, gripper_controller]

  arm_controller:
    action_ns: follow_joint_trajectory    # The ROS 2 action to call
    type: FollowJointTrajectory           # Standard trajectory execution
    joints: [panda_joint1, ..., panda_joint7]

  gripper_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints: [panda_finger_joint1, panda_finger_joint2]
```

#### `joint_limits.yaml` — Speed and acceleration caps

```yaml
default_velocity_scaling_factor: 0.1       # 10% of max speed globally
default_acceleration_scaling_factor: 0.1   # 10% of max acceleration

joint_limits:
  panda_joint1:
    has_velocity_limits: true
    max_velocity: 10.0          # rad/s
    has_acceleration_limits: true
    max_acceleration: 5.0       # rad/s²
  # ... same for all 9 joints (7 arm + 2 finger) ...
```

---

## 5. DATA FLOW — How Everything Connects

```
┌─────────────────────────────────────────────────────────────────────┐
│                        IGNITION GAZEBO                              │
│  ┌──────────┐   ┌──────────────┐   ┌──────────────────────────┐    │
│  │  World   │   │  Panda Robot │   │  Camera Sensor Plugin    │    │
│  │ (tables, │   │  (physics    │   │  → publishes to          │    │
│  │  blocks) │   │   joints)    │   │  /camera/image_raw       │    │
│  └──────────┘   └──────┬───────┘   └───────────┬──────────────┘    │
│                        │                        │                   │
│              ign_ros2_control              ign_ros2_control         │
└────────────────────────┼────────────────────────┼───────────────────┘
                         │                        │
                ┌────────▼────────┐     ┌─────────▼──────────┐
                │ Controller      │     │  Color Detector     │
                │ Manager         │     │  (panda_vision)     │
                │ ┌─────────────┐ │     │                     │
                │ │arm_controller│ │     │ HSV → Contour →    │
                │ │gripper_ctrl │ │     │ 3D Point → TF →    │
                │ │joint_state  │ │     │ Publish to          │
                │ │_broadcaster │ │     │ /color_coordinates  │
                │ └─────────────┘ │     └─────────┬──────────┘
                └────────┬────────┘               │
                         │                        │
                   /joint_states           /color_coordinates
                         │                        │
               ┌─────────▼──────────┐    ┌────────▼───────────┐
               │ robot_state_       │    │  pick_and_place    │
               │ publisher          │    │  (pymoveit2)       │
               │                    │    │                     │
               │ URDF + joint_states│    │ Subscribes to      │
               │ → TF transforms   │    │ coordinates →      │
               └─────────┬──────────┘    │ Calls MoveIt2 →   │
                         │               │ Plans + Executes   │
                    TF2 transforms       └────────┬───────────┘
                         │                        │
                         ▼                        ▼
               ┌──────────────────────────────────────────────┐
               │              MoveIt 2 (move_group)           │
               │                                              │
               │  Receives: pose goal / joint goal            │
               │  Does:     IK solve → motion plan → execute  │
               │  Uses:     SRDF, kinematics, joint limits    │
               │  Sends:    FollowJointTrajectory action to   │
               │            arm_controller / gripper_controller│
               └──────────────────────────────────────────────┘
```

---

## 6. HOW TO RUN THE PROJECT FROM SCRATCH

### Step 1: Prerequisites

```bash
# Ubuntu 22.04 with ROS 2 Humble installed
# Ignition Fortress (or Gazebo Harmonic, depending on your setup)

# Install required packages
sudo apt update
sudo apt install ros-humble-moveit ros-humble-moveit-configs-utils \
  ros-humble-moveit-ros-move-group ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-ros2-control ros-humble-ign-ros2-control \
  ros-humble-cv-bridge ros-humble-tf-transformations \
  ros-humble-joint-state-publisher-gui \
  python3-opencv python3-transforms3d
```

### Step 2: Create Workspace and Clone

```bash
mkdir -p ~/pandaws/src
cd ~/pandaws/src
git clone https://github.com/GSR07/pandaws.git .
# OR copy the packages manually
```

### Step 3: Build

```bash
cd ~/pandaws
colcon build --symlink-install
source install/setup.bash
```

### Step 4: Run the Full Demo

```bash
# Terminal 1: Launch everything
ros2 launch panda_bringup pick_and_place.launch.py
```

This single command starts: Gazebo (with world + robot) → Controllers → MoveIt → Vision → Pick-and-Place.

### Step 5: Change Target Color

Edit the launch file or pass as parameter:
```bash
# In pick_and_place.launch.py, change:
# {"target_color": "B"}  →  {"target_color": "R"} or "G"
```

---

## 7. DEBUGGING & VERIFICATION COMMANDS

```bash
# Check all running nodes
ros2 node list

# Check all active topics
ros2 topic list

# Monitor vision output
ros2 topic echo /color_coordinates

# Monitor joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames
# Opens a PDF showing the complete transform tree

# Check controller status
ros2 control list_controllers

# Manually test arm movement
ros2 run pymoveit2 ex_joint_goal.py --ros-args \
  -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"

# Manually test gripper
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"

# Test slider control (manual joint movement)
ros2 launch panda_controller slider_controller.launch.py
```

---

## 8. DEMONSTRATION TIPS

### Before the Demo

1. **Test everything privately first** — Run the full launch at least 3 times. Note how long Gazebo takes to load (can be 30-60 seconds).
2. **Pre-build** — Run `colcon build` beforehand so you don't wait during the demo.
3. **Source the workspace** — Add `source ~/pandaws/install/setup.bash` to your `~/.bashrc` so you don't forget.
4. **Have backup terminals ready** — Open 3-4 terminal tabs pre-configured.
5. **Know your system limits** — MoveIt + Gazebo + RViz is very resource-intensive. Close unnecessary applications. 16GB RAM is recommended.

### During the Demo

1. **Start with the architecture diagram** — Before showing code, explain the data flow (Section 5 above). Draw it on a whiteboard or show a slide.

2. **Demo in layers, not all at once:**
   - **Layer 1**: Show the URDF and Gazebo world. "This is the robot's body. These are the objects."
   - **Layer 2**: Show RViz and the TF tree. "These are the coordinate frames."
   - **Layer 3**: Use the slider controller to move joints manually. "This is how we control individual joints."
   - **Layer 4**: Show the vision node output. "The camera sees these colors and calculates 3D positions."
   - **Layer 5**: Run the full pick-and-place. "Now everything works together."

3. **Have a fallback plan** — If the full launch crashes (Gazebo can be flaky), show a pre-recorded video of a successful run. Screen-record a successful run beforehand.

4. **Common demo-day issues and fixes:**
   - **Gazebo doesn't load**: Kill all gazebo processes (`killall -9 ign gazebo ruby`), try again.
   - **Robot falls through table**: Check that the world file is correct.
   - **MoveIt fails to plan**: The start state might be in collision. Manually move to a safe state first.
   - **Vision doesn't detect objects**: Check camera topic: `ros2 topic hz /camera/image_raw`. Verify lighting in Gazebo world.
   - **Controllers fail to spawn**: Ensure you have `ign_ros2_control` installed and the URDF has the correct hardware interface.

5. **Key phrases to use during explanation:**
   - "The URDF describes WHAT the robot looks like. MoveIt describes HOW it should move."
   - "ros2_control is the bridge between the software commands and the simulated motors."
   - "The vision node converts 2D pixel coordinates to 3D world coordinates using the camera model and TF transforms."
   - "MoveIt uses RRT (Rapidly-exploring Random Trees) to find a path that avoids obstacles."

### Questions You Might Get (and Answers)

**Q: Why not use a depth camera instead of assuming Z=0.1?**
A: Great question. In a real project you would use an RGB-D camera (like Intel RealSense). This project simplifies by assuming a fixed depth for demonstration purposes.

**Q: Why is the motion so slow?**
A: The velocity scaling is set to 10% (`max_velocity = 0.1`) for safety. In a real deployment, you'd increase this after testing.

**Q: Why use joint-space moves for home/start/drop instead of Cartesian?**
A: Joint-space moves are more reliable because they don't require solving inverse kinematics. They guarantee reaching the exact configuration. Cartesian moves can fail if the path passes through a singularity or collision.

**Q: What is a quaternion?**
A: A quaternion is a 4-number representation of 3D orientation. `[0, 1, 0, 0]` means the gripper is pointing straight down (180° rotation around the Y axis). It's used instead of Euler angles because it avoids gimbal lock.

**Q: Can this work on a real robot?**
A: Yes! The architecture is designed for sim-to-real transfer. You would change `is_sim:=False`, replace `ign_ros2_control` with real hardware drivers, and calibrate the camera parameters.

---

## 9. GLOSSARY OF KEY TERMS

| Term | Meaning |
|---|---|
| **URDF** | XML file describing robot body (links + joints) |
| **Xacro** | Macro language for URDF (variables, conditionals, includes) |
| **SRDF** | Semantic Robot Description (groups, named poses, collision matrix) |
| **TF / TF2** | Coordinate transform library (tracks where every frame is) |
| **ros2_control** | Hardware abstraction + controller framework |
| **MoveIt 2** | Motion planning framework (IK, path planning, execution) |
| **IK** | Inverse Kinematics — compute joint angles from end-effector pose |
| **FK** | Forward Kinematics — compute end-effector pose from joint angles |
| **RRT** | Rapidly-exploring Random Trees — motion planning algorithm |
| **KDL** | Kinematics and Dynamics Library — IK solver used by MoveIt |
| **HSV** | Hue-Saturation-Value — color space used for color detection |
| **Quaternion** | 4D representation of 3D rotation (avoids gimbal lock) |
| **Action** | ROS 2 async request-response with feedback (for long tasks) |
| **Topic** | ROS 2 one-way data channel (publish/subscribe) |
| **Node** | ROS 2 process that does one specific task |
| **End Effector** | The "hand" at the tip of the robot arm |
| **Planning Scene** | MoveIt's internal 3D model of the environment for collision checking |
| **Callback Group** | ROS 2 mechanism controlling how callbacks execute (serial vs parallel) |
