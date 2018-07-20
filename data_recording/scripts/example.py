#! /usr/bin/env python
# Example of how to use the recording services
# This assumes other relevant nodes you want to record are launched
import rospy

from std_srvs.srv import Trigger, TriggerRequest
from visualization_msgs.msg import Marker


def something_interesting():
    """Dummy thing that takes up time"""
    global annotation_pub

    rospy.sleep(4)

    marker1 = create_text_marker("hello world!", [1, 1, 0])
    annotation_pub.publish(marker1)

    rospy.sleep(3)
    marker2 = create_text_marker("I'm another marker", [0, 0, 1], scale=[0.2, 0.2, 0.2], color=[0, 1, 0, 1],
                                 duration=[2, 500])
    annotation_pub.publish(marker2)

    rospy.sleep(1)


def create_text_marker(text, position, orientation=[0, 0, 0, 1], frame_id='/world', scale=[0.3, 0.3, 0.3],
                       color=[1.0, 0.0, 0.0, 1.0], duration=[1, 0]):
    """Create and populate a message of type visualization_msgs.msg/Marker

    :param text: the text you want
    :param position: [x, y, z] in the frame 'frame_id'
    :param orientation: [x, y, z, w] in the frame 'frame_id'
    :param frame_id:
    :param scale:
    :param color: [r, g, b, a]. Make sure a is not set to zero otherwise it will be invisible
    :param duration:
    """
    text_marker = Marker()
    text_marker.header.stamp = rospy.get_rostime()
    text_marker.header.frame_id = frame_id # You can also put text on say the end effector!
    text_marker.type = 9  # this specifies that the marker is a text marker
    text_marker.pose.position.x = position[0]
    text_marker.pose.position.y = position[1]
    text_marker.pose.position.z = position[2]
    text_marker.pose.orientation.x = orientation[0]
    text_marker.pose.orientation.y = orientation[1]
    text_marker.pose.orientation.z = orientation[2]
    text_marker.pose.orientation.w = orientation[3]
    text_marker.scale.x = scale[0]
    text_marker.scale.y = scale[1]
    text_marker.scale.z = scale[2]
    text_marker.color.r = color[0]
    text_marker.color.g = color[1]
    text_marker.color.b = color[2]
    text_marker.color.a = color[3] # make sure 'a' is set to >0, otherwise invisible
    text_marker.lifetime.secs = duration[0]
    text_marker.lifetime.nsecs = duration[1]
    text_marker.text = text

    return text_marker

if __name__ == '__main__':
    rospy.init_node('recording_example')
    rospy.loginfo('starting recording example')

    start_record_srv = rospy.ServiceProxy('/data_recording/start_recording', Trigger)
    stop_record_srv = rospy.ServiceProxy('/data_recording/stop_recording', Trigger)

    annotation_pub = rospy.Publisher('/data_recording/annotations', Marker, queue_size=10)

    start_record_srv(TriggerRequest())
    # now do something we want to record
    try:
        something_interesting()
    except Exception as e:
        stop_record_srv(TriggerRequest())
        raise e
    stop_record_srv(TriggerRequest())

    rospy.loginfo('Example code has completed, please refer to README on how to replay the data')