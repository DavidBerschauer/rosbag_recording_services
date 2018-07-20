# Data Recording
This is a simple ROS package that allows starting and stopping rosbag recordings via service calls, 
which is useful for performing repeated data collection. It also allows replaying those recordings for
analyzing performance.

## Run
`roslaunch data_recording data_recording.launch` uses the default mode:=minimal.  

## Config
The config files for the recording are in the config directory. You can choose between different settings and add new config files.
Configure the output directory for the rosbag files and topics to be recorded (regex syntax is ok).
Rosbag files are saved with the current date and time as a filename.

## Usage

The package creates three services:
* `/data\_recording/start\_recording` which can be called with a `std_srvs.srv.RecordRequest` message
* `/data\_recording/stop\_recording` which can be called with a `std_srvs.srv.TriggerRequest` message
* `/data\_recording/toggle\_recording` which can be called with a `std_srvs.srv.TriggerRequest` message

Add the following imports to your ROS node:

```python
from std_srvs.srv import Trigger, TriggerRequest
from data_recording.srv import Record, RecordRequest
```

In the init of your ROS node, launch the services as
```python
start_record_srv = rospy.ServiceProxy('/data_recording/start_recording', Record)
stop_record_srv = rospy.ServiceProxy('/data_recording/stop_recording', Trigger)
```

Then in a regular service call:

```python
start_record_srv(RecordRequest('name of your file')  # this allows you to modify name based on e.g. the run
do_some_stuff()
stop_record_srv(TriggerRequest())
```

If you leave the `bagname` field of the `RecordRequest` empty, the file will be named using 
the datetime of the recording.

### Annotations

From within the code you can add annotations to the stored rosbags by publishing 
strings to a special topic `/data_recording/annotations`. These messages are then displayed on Rviz
so you can analyze important events that happened in the code. This feature is optional.

Simply create a publisher in your initialization: 
```python
annotation_pub = rospy.Publisher('/data_recording/annotations', Marker, queue_size=10)
```
and add `from visualization_msgs.msg import Marker` to your imports.

Then create a text marker. See `example.py` for a function that does this.

Publish an annotation any time anywhere using `annotation_pub.publish(text_marker)`

If you want to test this functionality, launch the data_recording services as outlined above and
run `rosrun data_recording example.py`.

## Replaying a rosbag

Kill all ROS nodes currently running. This file will launch all necessary nodes and rviz for you.

```roslaunch data_recording replay.launch filename:=YOUR_RECORDING```

This launch file assumes a specific path for the recordings which you can
specify using `filepath:=YOUR_FILEPATH`.

This launch file launches a specific rviz config file. You can make a new file, store it in the config folder
and load it using the `mode` argument.

