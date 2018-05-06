# drone-orientation-estimation
A python's implementation of **Tracking Drone Orientation with Multiple GPS Receivers**  
http://synrg.csl.illinois.edu/papers/drone-attitude-camera.pdf

Vector Representation of Rotation:
https://www2.cs.duke.edu/courses/compsci527/fall13/notes/rodrigues.pdf

ATTITUDE
https://diydrones.com/forum/topics/open-source-attitude

Rotation Vector:
http://farside.ph.utexas.edu/teaching/301/lectures/node100.html

Vector Cross Product:
http://soe.rutgers.edu/%7emeer/GRAD561/ADD/antisymm.pdf

## Simulator Steps:
- `source devel/setup.bash`
- launch the drone with `roslaunch`
- enable the motor `rosservice call /enable_motors "enable: true"`
- start keyboard controller `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
- start data subscriber `rosrun pose_subscriber pose_listener.py`
