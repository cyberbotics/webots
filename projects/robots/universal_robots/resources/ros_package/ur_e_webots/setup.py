from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ur_e_webots'],
    package_dir={'': 'src'},
    requires=['roslib', 'rospy', 'sensor_msgs', 'actionlib', 'actionlib_msgs', 'control_msgs', 'trajectory_msgs']
)

setup(**d)
