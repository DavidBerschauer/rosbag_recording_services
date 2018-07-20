# README
This is a simple ROS package that allows starting and stopping rosbag recordings via service calls, which is useful for performing repeated data collection.

## Run
`roslaunch data_recording data_recording.launch` uses the default mode:=minimal.  

## Config
The config files for the recording are in the config directory. You can choose between different settings and add new config files.
Configure the output directory for the rosbag files and topics to be recorded (regex syntax is ok).
Rosbag files are saved with the current date and time as a filename.

## Usage

The package creates three services, which can be called with a `std_srvs.srv.TriggerRequest` message:
* /data\_recording/start\_recording
* /data\_recording/stop\_recording
* /data\_recording/toggle\_recording

Add the following import to your ROS node:

`from std_srvs.srv import Trigger`

In the init of your ROS node, launch the services as
```python
start_record_srv = rospy.ServiceProxy('/data_recording/start_recording', Trigger)
stop_record_srv = rospy.ServiceProxy('/data_recording/stop_recording', Trigger)
```

Then in a regular service call:

```python
start_record_srv(std_srvs.srv.TriggerRequest())
do_some_stuff()
stop_record_srv(std_srvs.srv.TriggerRequest())
```

### Annotations

From within the code you can add annotations to the stored rosbags by publishing 
strings to a special topic `/data_recording/annotations`. 
Simply create a publisher in your initialization: 
```python
annotation_pub = rospy.Publisher('/data_recording/annotations', std_msgs.msg.String, queue_size=10)
```

Publish an annotation any time anywhere using `annotation_pub.publish(std_msgs.msg.String("foo"))`
