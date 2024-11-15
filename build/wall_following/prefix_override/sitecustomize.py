import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/users/956024/ir/ros/install/wall_following'
