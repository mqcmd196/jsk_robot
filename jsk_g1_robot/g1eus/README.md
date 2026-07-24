# g1eus

![g1eus](./figs/g1eus.png)

`g1eus` provides a ROS 2 roseus interface and EusLisp model for the Unitree G1.
The EusLisp model `g1.l` is committed in this package and is not generated at
build time.

## Setup

Install ROS 2 Jazzy and workspace tools first.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y python3-rosdep python3-vcstool python3-colcon-common-extensions
```

Create a colcon workspace and import the required source repositories. This
includes ROS 2 roseus from `jsk_roseus`, the G1 ROS 2 driver, and
`inspire_tutorials` under `jsk_g1_robot/`.

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
wget https://raw.githubusercontent.com/Michi-Tsubaki/jsk_robot/refs/heads/ros2/jsk_g1_robot/g1eus/jazzy.repos -O g1eus.jazzy.repos
vcs import < g1eus.jazzy.repos
```

To update an existing workspace:

```bash
cd ~/colcon_ws/src
vcs pull
```

Install binary dependencies and build.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws
rosdep update
rosdep install -iqry --from-paths src --ignore-src
colcon build --symlink-install --packages-up-to g1eus g1_bringup
source install/setup.bash
```

## Kinematic Simulator

The interface can run without a real controller by using its kinematic simulator
mode.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
roseus
```

```lisp
(load "package://g1eus/g1-interface.l")
(g1-init :execution-mode :kinematic-simulator)
(send *ri* :angle-vector (send *g1* :angle-vector) 1000)
(send *ri* :wait-interpolation)
```

With `:execution-mode :auto`, `g1-interface.l` uses the ROS 2 controller when it
is available and falls back to the kinematic simulator when it is not.

## Real Robot: Default ROS 2 Mode

This section is for the normal `g1_bringup` path, not SONIC. Do not pass
`use_gear_sonic:=true`.

If the robot has been running the SONIC systemd stack, stop it on the robot
before starting the default ROS 2 bringup. The SONIC bringup service starts a
containerized `g1_bringup` with `use_gear_sonic:=true` and requires
`gear-sonic.service`, so leaving it active conflicts with the default bringup.

```bash
ssh unitree@192.168.123.164
sudo systemctl stop ros2-g1-gear-sonic-bringup.service gear-sonic.service
sudo systemctl status ros2-g1-gear-sonic-bringup.service gear-sonic.service --no-pager
```

If these units are not found, the SONIC systemd stack is not installed on that
robot. For default ROS 2 mode, do not start
`ros2-g1-gear-sonic-bringup.service`.

The Inspire hand service is independent of SONIC. If you use Inspire RH56DFX
hands, keep `inspire-g1.service` running, or start the hand service manually
when systemd is not installed.

```bash
sudo systemctl start inspire-g1.service
sudo systemctl status inspire-g1.service --no-pager
```

Manual hand-service startup:

```bash
ssh unitree@192.168.123.164
cd ~/dfx_inspire_service/build
sudo ./inspire_g1 -k -u
```

On the ROS 2 computer, connect Ethernet to G1, set the network interface as
described in the Unitree G1 developer instructions, and launch the default
bringup.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network_interface>
```

With Inspire RH56DFX hands:

```bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network_interface> hand_type:=inspire_dfq
```

By default, the hands close when their hardware interface is deactivated. To
disable that behavior:

```bash
ros2 launch g1_bringup g1_bringup.launch.py network_interface:=<network_interface> hand_type:=inspire_dfq close_hand_on_deactivate:=false
```

Activate the upper-body trajectory controller:

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 control set_controller_state upper_body_controller active
```

Run roseus:

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
roseus
```

```lisp
(load "package://g1eus/g1-interface.l")
(g1-init :execution-mode :controller)
(send *ri* :wait-for-server 10.0)
(send *ri* :angle-vector (send *g1* :angle-vector) 3000)
(send *ri* :wait-interpolation)
```

## Command Velocity

In the default ROS 2 mode, `g1_hardware` starts `loco_cmd_adapter`, which
subscribes to `/cmd_vel`. `g1-interface.l` publishes `geometry_msgs/Twist` to
`/cmd_vel` by default.

```lisp
(send *ri* :cmd-vel 0.1 0.0 0.0)  ;; x, y, yaw
(send *ri* :cmd-vel 0.0 0.1 0.0)
(send *ri* :cmd-vel 0.0 0.0 0.2)
(send *ri* :stop-base)
```

The topic can be changed when initializing:

```lisp
(g1-init :cmd-vel-topic "/cmd_vel")
```

## Hand Interface

```lisp
(send *ri* :hand-angle-vector :larm #f(0 0 0 0 0 0) 1000)

(send *ri* :stop-grasp :larm)
(send *ri* :prepare-grasp :larm)
(send *ri* :start-grasp :larm)
(send *ri* :default-grasp :larm)
```

The right hand uses `:rarm` in the same way.
