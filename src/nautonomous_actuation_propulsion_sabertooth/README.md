# nautonomous propulsion sabertooth {#nautonomous_propulsion_sabertooth}

sabertooth_propulsion_node starts the software

sabertooth_propulsion is the main object the coordinates the tasks of reading, processing and writing information.

sabertooth_packet is a packet that encapsulates the address, command, value and checksum

sabertooth_motor_driver processes a message (twist or left/right) to create a sabertooth packet

sabertooth_serial sends the packet using the serial driver to the serial port.

serial.cpp Serial functions.

actuation_watchdog processes input from the actuation platform to check if the actuation platform is responding correctly.

## Nodes
nautonomous_actuation_propulsion_sabertooth 

##Topics

###Subscribe
left_motor_topic -> motor/left
right_motor_topic -> motor/right
cmd_vel_topic -> cmd_vel

###Publish
left_feedback_topic -> feedback/left
right_feedback_topic -> feedback/right

## Files
[Include](dir_24f9e5a3f98ba5b1652caba176a493c4.html)  |  [Src](dir_0e1bf686876a66f778c9a6403953ffcc.html)

##Overview
![send_actuation.launch](../images/launch_send_actuation.png)
<br />
![legenda](../images/legenda.png)

