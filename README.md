# drone-orientation-estimation
A python's implementation of **Tracking Drone Orientation with Multiple GPS Receivers**  
http://synrg.csl.illinois.edu/papers/drone-attitude-camera.pdf

## Simulator Steps:
- `source devel/setup.bash`
- launch the drone with `roslaunch`
- enable the motor `rosservice call /enable_motors "enable: true"`
- start keyboard controller `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
- start data subscriber `rosrun pose_subscriber pose_listener.py`
