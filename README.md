# omniradar
ROS Interface for Omniradar's RIC-60 Radar Development Kit

`omniradar` is a ROS package which uses [libomniradar](https://github.com/lalten/libomniradar) to read data from an [Omniradar RIC60](http://www.omniradar.com/products/) UWB FMCW radar sensor.

The radar signals are published as [RadarEcho](https://github.com/lalten/omniradar/blob/master/msg/RadarEcho.msg) rostopic.
The `omniradar` node also provides a Dynamic Reconfigure server which allows to set number of sweeps, sweep duration, and RIC config string.

Run the node with `roslaunch omniradar omniradar.launch`.

Example output:

```
[ INFO] [1505350814.207264375] [/omniradar_node]: RDK ready
[ INFO] [1505350825.783483622] [/omniradar_node]: Set sweep to 1 x 20ms
```
