# kimm_polaris3d_moveit
KIMM polaris3d application with Padna Robot Arm & Moveit

## 1. Prerequisites
### 1.1 Moveit configuration
```bash
git clone https://github.com/jspark861102/panda_moveit_config.git.git -b noetic-devel
```

### 1.2 Robot & Gripper model
```bash
#git clone https://github.com/jspark861102/franka_ros.git #my used version (0.8.1)
git clone https://github.com/frankaemika/franka_ros.git #my used version (0.10.1)
git clone https://github.com/jspark861102/kimm_robotiq.git #pymodbuss<2.5.3 required
git clone https://github.com/jspark861102/kimm_robots_description.git -b melodic
```
### 1.3 polaris3d beverage delivery task
```bash
git clone https://github.com/jspark861102/kimm_polaris3d_moveit.git
```

## 2. Run
```bash
# beverage delivery task
roslaunch kimm_polaris3d_moveit move_to_start.launch
python3 /scripts/move_tostart.py
```