import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/retina/md_ws/src/MD_controller_ROS2/install/md_teleop'
