# Stretch IK

Inverse Kinematics for the Hello Robot Stretch... nice!

## Contact

Matt Lamsey - lamsey [AT] gatech [DOT] edu

# Installation

The usual:

```bash
cd ~/catkin_ws/src
git clone https://github.com/mlamsey/stretch_ik.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Dependencies

Depends on `ikpy` - TODO: install commands

## Making a URDF for `ikpy`

The `urdf` folder contains a simplified URDF for `ikpy` to use. If you want to regenerate it (e.g. if you calibrated your Stretch URDF), follow these steps:

1. `cd` into `/src`

2. `python urdf_ripper.py --urdf_path /path/to/original/urdf/stretch.urdf --output_path /path/to/output/urdf/stretch.urdf`

The `urdf_ripper.py` script will remove all the unnecessary links and joints from the original URDF, and save it to the output path. It's based on [this tutorial](https://github.com/hello-robot/stretch_tutorials/blob/master/stretch_body/jupyter/inverse_kinematics.ipynb).

# Usage

Example python usage:
```python
from stretch_ik.ik import StretchDexIK

# Initialize IK engine
urdf_path = "/path/to/urdf/stretch.urdf"
self.ik_engine = StretchDexIK(urdf_path)

# Solve IK
target_pos = [0.5, -0.4, 0.2]
target_ori = [[1., 0., 0.],
              [0., 1., 0.],
              [0., 0., 1.]]

pose_dict = ik_engine.solve_ik(target_pos, target_ori)
```

Example integration with Stretch's ROS driver:

```python
import rospy
import hello_helpers.hello_misc as hm

class StretchNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'stretch_controller', 'stretch_namespace', wait_for_first_pointcloud=False)
        self.move_to_pose(pose_dict)
```

## Demo

Terminal 1:

`roslaunch stretch_ik strech_core.launch`

Terminal 2:

`rosrun stretch_ik demo.py`

## Tips

Some miscellaneous tips for using this package:

### Incorporate `/tf`

Consider using `/tf` to look up transformations from the robot's base link to a target, e.g.

```python
import tf
from stretch_ik.ik import StretchDexIK
from stretch_ik.utils import quaternion_to_rotation_matrix

# Initialize IK engine
urdf_path = "/path/to/urdf/stretch.urdf"
self.ik_engine = StretchDexIK(urdf_path)

# Initialize tf listener
tf_listener = tf.TransformListener()

# Get target pose
transform = tf_listener.lookupTransform("base_link", "robot_target", rospy.Time(0.))
pos = transform[0]
ori = transform[1]
ori_rotmat = quaternion_to_rotation_matrix(ori)

# Solve IK
pose_dict = ik_engine.solve_ik(pos, ori_rotmat)
```
### Add Navigation

E.g. point-to-point driving, or use the nav stack
