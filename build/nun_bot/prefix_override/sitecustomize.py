import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/labtech/ros2_ws/src/nun_bot/install/nun_bot'
