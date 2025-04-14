import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yahboom/Desktop/ROB2/finalproject_Team5_ws/install/multirobotctrl'
