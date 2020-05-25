 ![](doc/images/logo.png)

# G-Plore 
G-Plore is ROS-based self-driving software, enabling self-driving vehicles to be in private areas, public roads,Mining area.  The current target of G-Plore provides, but not limited to, the functional modules described below.
* ***Localization*** depends on RTK device, using odometry information obtained from CAN messages and GNSS/IMU sensors.
* ***Perception*** is empowered by Radar and LiDAR devices in combination with high-definition map data. The Detection module uses deep learning and sensor fusion approaches.Tracking and Prediction are realized with the Kalman Filter algorithm and the lane network information provided by high-definition map data.
* ***Planning*** is based on sampling search and rule-based systems.
* ***Control*** defines motion of the vehicle with a twist of velocity and angle (also curvature). The Control module falls into both the G-Plore-side stack (Stanly and Pure Pursuit) and the vehicle-side interface (PID variants).

To conclude, G-Plore will provides a complete software stack for self-driving vehicles. Let's work hard together, and your contribution will be loved by the world.
## Prerequisites

### Basic Requirement

* A machine with a 4-core processor and 8GB memory minimum

* Ubuntu 18.04
* ROS melodic
* Working knowledge of ROS
  
### Preparation
**Step 1.** pull from git clone git@gitee.com:glb-auto_sh/G-Plore.git ,the command
```shell
    git clone git@gitee.com:glb-auto_sh/G-Plore.git
```

**Step 2.** install development dependency
```shell
  cd script
  sudo ./install_SF.sh
```

**Step 3.** for prevent build-time cross-talk and clean code ,suggest to install catkin tool,open your termiantor, and run

```shell
    sudo apt install python-catkin_tool.sh
```

##  Building and Running

steps:
1. first time you need change directory to admsystem, run command 
```shell
    catkin build
```
2. if you want to build and debug your package, only run the command
```shell
    catkin clean "your package name"
    catkin build "your package name"
```
3. launch your node

## Follow below commit message format

```
[Your Topic] Short Description of the change

More detail description

Signed-off-by: your name <your email>
```
