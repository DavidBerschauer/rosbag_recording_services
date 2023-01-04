#!/usr/bin/env python3

import os
import pathlib
import subprocess

import rospy
import rosnode
from std_srvs.srv import Trigger, TriggerResponse
from datetime import datetime, timedelta
from data_recording.srv import Record, RecordResponse, RecordState, RecordStateResponse


class DataRecorder():
    def __init__(self):
        self.start_recording_service = rospy.Service('/data_recording/start_recording', Record, self.start_recording)
        self.stop_recording_service = rospy.Service('/data_recording/stop_recording', Trigger, self.stop_recording)
        self.toggle_recording_service = rospy.Service('/data_recording/toggle_recording', Trigger, self.toggle_recording)
        self.recording_state_service = rospy.Service('/data_recording/recording_state', RecordState, self.recording_state)
        
        self.process = None
        self.recording = False
        self.timeout_timer = None
        self.filename = ''

        out_param = rospy.get_param('/data_recording/output_directory', '../rosbags/')
        if out_param.startswith('..'):
            self.output_directory = os.path.join(pathlib.Path(__file__).parent.resolve(), out_param)
        else:
            self.output_directory = out_param

        self.topics = rospy.get_param('/data_recording/topics', [])
        if not self.topics:
            rospy.logerr('No Topics Specified.')

        rospy.loginfo('Data Recorder Started\nLogging into ' + self.output_directory)
        

    def toggle_recording(self, req):
        if self.recording:
            return self.stop_recording(req)
        else:
            return self.start_recording(req)

    def timeout(self, event):
        """Stop recording when the timeout is reached, callback function"""
        if self.recording:
            rospy.logwarn("Reached recording limit, stopping recording!")
            self.stop_recording(Trigger())

    def start_recording(self, req):
        if self.recording:
            rospy.logerr('Already Recording')
            return RecordResponse(False, 'Already Recording')

        self.topics = rospy.get_param('/data_recording/topics', [])
        if not self.topics:
            rospy.logerr('No Topics Specified.')

        if req.timeout > 0:
        # start a timer to limit record time
            self.timeout_time = datetime.now() + timedelta(0, req.timeout)
            self.timeout_timer = rospy.Timer(rospy.Duration(req.timeout), self.timeout, oneshot=True)
        else:
            self.timeout_timer = None

        if req.bagname != '':
            self.filename = req.bagname
        else:
            self.filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        command = ['rosrun', 'rosbag', 'record', '-O', self.filename, '-e'] + self.topics + \
                      ['__name:=data_recording_myrecorder']
        
        self.process = subprocess.Popen(command, cwd=self.output_directory)
        self.recording = True
        rospy.loginfo('Started recorder, PID %s' % self.process.pid)
        return RecordResponse(True, 'Started recorder: ' + self.filename)

    def stop_recording(self, req):
        if not self.recording:
            rospy.logerr('Not Recording')
            return TriggerResponse(False, 'Not Recording')

        bag_size = self.get_bag_size()
        rosnode.kill_nodes(['/data_recording_myrecorder'])

        self.process = None
        self.recording = False
        if self.timeout_timer is not None:
            self.timeout_timer.shutdown()

        rospy.loginfo('Stopped Recording')
        return TriggerResponse(True, f'Stopped Recording {bag_size}')

    def recording_state(self, req):
        if self.timeout_timer is not None and self.timeout_timer.is_alive():
            remaining = f'{(self.timeout_time - datetime.now()).total_seconds():.1f}'
        else:
            remaining = '-'
        return RecordStateResponse(self.recording, 
                                    self.filename,
                                    self.get_bag_size(),
                                    self.topics,
                                    remaining)

    def get_bag_size(self) -> str:
        try:
            try:
                file_stats = os.stat(os.path.join(self.output_directory, self.filename + '.bag.active'))
            except:
                file_stats = os.stat(os.path.join(self.output_directory, self.filename + '.bag'))
            if(file_stats.st_size > 1000000000):
                return f'{file_stats.st_size/(1000000000):.2f} GB'
            if(file_stats.st_size > 1000000):
                return f'{file_stats.st_size/(1000000):.2f} MB'
            if(file_stats.st_size > 1000):
                return f'{file_stats.st_size/(1000):.2f} KB'
            return f'{file_stats.st_size} Byte'
        except:
            return 'File not found'

if __name__ == "__main__":
    rospy.init_node('data_recording')
    DataRecorder()
    rospy.spin()
