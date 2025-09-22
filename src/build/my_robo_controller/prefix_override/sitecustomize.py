import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/bishaldas/Apps/Ros2_4wheel_Control_bot/src/install/my_robo_controller'
