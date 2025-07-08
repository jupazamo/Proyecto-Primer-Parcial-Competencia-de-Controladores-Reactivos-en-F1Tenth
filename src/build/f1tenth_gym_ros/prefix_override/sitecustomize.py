import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jupazamo/F1Tenth-Repository/src/install/f1tenth_gym_ros'
