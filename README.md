# Drones

In order to make this project work

##System Requirements:

Ubuntu 16.04

Ros Kinetic

##Actions:

Control_robot.action: This action is used to go to a specific location already stored by the name.

go_to_marker.action: This action is used to go to a fixed marker location setup in the code.

go_to_target.action: This action is used to go to a specific user defined location.

patrol.action: This action is used to go to a go to all the location stored by the name sequentically.

##Services:

follow_moving.srv: This service is used to follow a moving marker. Here we have mimicked how a marker will move by periodically changing the marker location.

memorize_position.srv: This service is used to store current location of drone with a name.

take_off.srv: This service is used to take drone off the ground and hover at a specific height.

z_height.srv: This service is used to change the drone hover height after take_off.

##Scripts:

controller.py: This is the main node which initiates all the services and actions server.

control_robot_client.py: This is the client node to call control_robot.action

go_to_marker.py: This is the client node to call go_to_marker.action

go_to_target.py: This is the client node to call go_to_target.action

patrol_client.py: This is the client node to call patrol.action

memory_client.py: This is the client node to call memorize_position.srv
