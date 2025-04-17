import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yahboom/RoboticsII_final/finalproject_Team5_ws/install/multirobotctrl'
