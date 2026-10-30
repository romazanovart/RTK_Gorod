import os


#print('Поворот налево')


os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 3200, speed: 150}"')

os.system('ros2 service call /rotate_robot frob_interfaces/srv/Turn "{angle: -84, speed: 100}"')

os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 2400, speed: 150}"')

os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 1000, speed: 150}"')

os.system('ros2 service call /rotate_robot frob_interfaces/srv/Turn "{angle: -86, speed: 100}"')

os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 1700, speed: 150}"')

os.system('ros2 service call /rotate_robot frob_interfaces/srv/Turn "{angle: -67, speed: 100}"')

os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 1600, speed: 150}"')

os.system('ros2 service call /rotate_robot frob_interfaces/srv/Turn "{angle: -257, speed: 100}"')

os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 800, speed: 150}"')

os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 1000, speed: 150}"')

os.system('ros2 service call /rotate_robot frob_interfaces/srv/Turn "{angle: -87, speed: 100}"')

os.system('ros2 service call /forward_robot frob_interfaces/srv/Forward "{dist: 800, speed: 150}"')
