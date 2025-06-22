import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/henray/RoMaze_ws/src/fp_robot/install/fp_robot'
