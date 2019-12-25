# 3DLidar_curb_detection
1- download the rosbag for the VELODYNE 3D lidar
[click here](http://db3.ertl.jp/autoware/sample_data/sample_moriyama_150324.tar.gz)

```Bash
$ cd 
$ cp ~/Downloads/sample_moriyama_* .
$ tar zxfv sample_moriyama_150324.tar.gz

```

2- clone the repo into your workspace

```Bash
$ cd 
$ cd catkin_ws/src/
$ git clone https://github.com/SohaibAl-emara/3D_Lidar_Curb_Detection.git
```

3- build the code
```Bash
$ cd 
$ cd catkin_ws/
$ catkin_make
```
4- run rosmaster
```BASH
$ roscore
```

5- in a different terminal, play the rosbag 
```BASH
$ cd 
$ rosbag play sample_moriyama_150324.bag 
```

6- in a different terminal, open rviz
```
$ rosrun rviz rviz -f velodyne
```

7- in a different terminal, run the curb detection node
```Bash
$ rosrun curb_detec_cpp curb_detec
```

8- setup rviz to see the pointcloud2 data 

![alt text](https://imgshare.io/images/2019/12/25/Selection_001.png)

