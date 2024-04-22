# ROS2 Braccio Arm Topic Message and Custom Service

### Braccio Arm Setup 
Check the required packages already install, if not install them
```
sudo apt install -y ros-iron-joint-state-publisher-gui ros-iron-robot-state-publisher ros-iron-xacro python3-pip
```
Install following python3 packages
```
pip install inputimeout pyserial
```
Setup your VirtualBox USB port, (Right Click) Setting -> USB -> Arduino (Add). Modify the ttyACM0 to 777
```
sudo chmod 777 /dev/ttyACM0
```
Pull from repository
```
cd ~/dev_ws/src
git clone http://github.com/twming/ros2_training_braccio_arm
```
Edit 3 python .py files in ba_control/scripts, by removing the comment self.arm_serial. Save those files. Change the directory to your workspace
```
cd ~/dev_ws
```
Build the package 
```
colcon build
```
Source the environment
```
source install/setup.bash
```

### Braccio Arm Control through JointState Message
Control the arm movement from the GUI interface. First launch the arm, then run arm_move_msg.py to continuously move the arm
```
ros2 launch ba_description braccio_arm.launch
```
```
ros2 run ba_control arm_move_msg.py
```

### Braccio Arm Control through JointAngle Service
Run the arm_move_srv_server.py, follow by arm_move_srv_client.py
```
ros2 run ba_control arm_move_srv_server.py
```
```
ros2 run ba_control arm_move_srv_client.py
```
