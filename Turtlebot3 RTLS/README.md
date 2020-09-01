The Turtlebot3-based prototype has been equipped with a Real Time Localization System produced by Sewio. The differential drive vehicle will receive the location data from socket and use them to navigate in the indoor environment. Once the RTLS Studio is started, the node rtls_sewio, implemented through the rtls_sewio.py script, will read the localization data from socket and publish them in the topic /pose_rtls. The robot_localization package will allow the fusion of multiple sources of localization data by mean of Extended Kalman Filters, through which localization data coming from odometry, IMU and RTLS system are used to produce an optimal position estimation.

agv.launch provides:
1) initialize and start the navigation system;
2) start the nodes for reading Sewio rtls data from socket;
3) start the rviz visualization tool and the necessary markers
