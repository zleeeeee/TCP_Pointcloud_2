# Send pointcloud by tcp

## introduction
There are two demos that send pointcloud data by tcp. 

The fisrt demo zle_tcp_1, client send pointcloud by a pcd file, then server receive it and save it as a pcd file.

![markdown picture](/pcd.png)

The second demo send pointcloud、odom data used ros.

## zle_tcp_1
1.     cmake .
2.     make
3.     ./server
4.     ./client


## zle_tcp_ros_2
1.     catkin_make
2.     roslaunch message_receive.launch
3.     roslaunch message_send.launch
4. publish pointcloud topic or rosbag play your bag

![markdown picture](/demo.png)
![markdown picture](/rviz.png)


# TCP_Pointcloud
