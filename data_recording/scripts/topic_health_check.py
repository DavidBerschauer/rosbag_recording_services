#!/usr/bin/env python3

import rospy
import threading # Needed for Timer
from std_msgs.msg import String
import time

class TopicHealthCheck:
    def __init__(self):
        self.topic = rospy.get_param('~topic', '/topic')
        self.title = rospy.get_param('~title', 'Unknown')
        self.timeout = float(rospy.get_param('~timeout', '1'))
        self.warning_timeout = float(rospy.get_param('~warning_timeout', '1'))
        self.update_time = float(rospy.get_param('~update_time', '1'))
        self.hz_precision = int(rospy.get_param('~hz_precision', '0'))
        if(self.hz_precision < 0):
            self.hz_precision = 0
        #TODO Implement warning timeout
        self.status_pub = rospy.Publisher(self.topic + '/status', String, latch=True, queue_size=10)
        self.topic_sub = rospy.Subscriber(self.topic, rospy.AnyMsg, self.on_topic, queue_size=10)
        self.timer = threading.Timer(self.timeout,self.on_timeout) # If X seconds elapse, call timeout()
        self.timer.start()
        self.status_pub.publish(String(data=f'{{"title":"{self.title}","type":"N/A"}}'))
        self.last_update = time.time()
        self.had_event = False
        self.events_since_update = 0

    def on_topic(self, msg):
        self.had_event = True
        self.events_since_update += 1
        now = time.time()
        if((now - self.last_update) >= self.update_time):
            self.status_pub.publish(String(
                data=f'{{"title":"{self.title}","type":"OK", "hz":"{self.events_since_update/(now-self.last_update):.{self.hz_precision}f}"}}'))
            self.last_update = now 
            self.events_since_update = 0
        self.reset_timer()

    def on_timeout(self):
        self.events_since_update = 0
        self.last_update = time.time()
        if(self.had_event):
            self.status_pub.publish(String(data=f'{{"title":"{self.title}","type":"Timeout"}}'))
        else:
            self.status_pub.publish(String(data=f'{{"title":"{self.title}","type":"N/A"}}'))    
        self.reset_timer()

    def reset_timer(self):
        self.timer.cancel()
        self.timer = threading.Timer(self.timeout,self.on_timeout)
        self.timer.start()

if __name__ == '__main__':
    try:
        rospy.init_node('topic_health_check')
        p = TopicHealthCheck()
    except Exception as e:
        rospy.logerr(e)
    rospy.spin()