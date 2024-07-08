import os
import datetime
import time
import yaml

import subprocess

import socket
from xmlrpc.client import Fault

import rospy
import rosgraph

from .topichandler import TopicHandler

from PyQt5.QtCore import QObject, pyqtSignal, QTimer




def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        # TODO: remove, this is for 1.1
        # sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        rospy.logerr(
            "WARNING: rostopic is being used against an older version of ROS/roscore"
        )
        val = master.getPublishedTopics('/')
    return {t: t_type for t, t_type in val}


def _get_published_topics_types(master):
    try:
        state = master.getSystemState()
        pubs, _, _ = state
        pub_topics = [t for t, _ in pubs]
        topics_types = _master_get_topic_types(master)
    except socket.error:
        raise Exception("Unable to communicate with master!")

    return {t: topics_types[t] for t in pub_topics}


class GUIBackend(QObject):
    add_topics_signal = pyqtSignal(list)
    remove_topics_signal = pyqtSignal(list)
    check_topics_signal = pyqtSignal(list)

    started_recording_signal = pyqtSignal()
    stopped_recording_signal = pyqtSignal(str)

    status_message_signal = pyqtSignal(str)

    topics_stats_signal = pyqtSignal(dict)

    set_bag_savedir_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self._check_available_topics_timer = QTimer()
        self._check_available_topics_timer.timeout.connect(
            self._check_available_topics_callback
        )
        self._check_available_topics_timer.start(1000)

        self._update_topics_stats_timer = QTimer()
        self._update_topics_stats_timer.timeout.connect(
            self._update_topics_stats_callback
        )
        self._update_topics_stats_timer.start(1000)

        self._master_handle = rosgraph.masterapi.Master('/bag_gui')

        self._available_topics_types = {}
        self._topic_handlers = {}

        self._recording = False
        self._record_bag_process = None
        self._bag_save_folder = os.getcwd()
        self._bag_path = None

    def _check_available_topics(self):

        try:
            topics_types = _get_published_topics_types(self._master_handle)
        except socket.error:
            raise Exception("Unable to communicate with master!")

        added_topics = []
        for t, t_type in topics_types.items():
            if t not in self._available_topics_types:

                self._available_topics_types[t] = t_type
                added_topics.append(t)
        if len(added_topics) > 0:
            self.add_topics_signal.emit(added_topics)

        rm_topics = []
        for t in self._available_topics_types:
            if t not in topics_types:
                rm_topics.append(t)

                if t in self._topic_handlers:
                    self._remove_topic(t)

        for t in rm_topics:
            del self._available_topics_types[t]
        if len(rm_topics) > 0:
            self.remove_topics_signal.emit(rm_topics)

    def _check_available_topics_callback(self):
        if not self._recording:
            self._check_available_topics()

    def _update_topics_stats_callback(self):
        topics_stats = {t: h.get_stats() for t, h in self._topic_handlers.items()}
        self.topics_stats_signal.emit(topics_stats)

    def _remove_topic(self, topic):
        # NOTE for some reason del doesn't call the destructor, so we need to manually call unregister
        self._topic_handlers[topic]._subscriber.unregister()
        del self._topic_handlers[topic]

    def add_topic(self, topic):
        if self._recording:
            rospy.logwarn(f'Cannot add topic {topic} while recording')
            return
        if topic not in self._available_topics_types:
            rospy.logwarn(f'Cannot add topic {topic} which is not currently published')
            return
        self._topic_handlers[topic] = TopicHandler(
            topic, self._available_topics_types[topic]
        )

    def remove_topic(self, topic):
        if self._recording:
            rospy.logwarn(f'Cannot remove topic {topic} while recording')
            return
        if topic not in self._topic_handlers:
            rospy.logwarn(f'Cannot remove topic {topic} because it was never added')
            return
        self._remove_topic(topic)

    def start_recording(self):
        if len(self._topic_handlers) == 0:
            msg = 'Cannot start recording as no topics are selected'
            self.status_message_signal.emit(msg)
            rospy.logwarn(msg)
            return

        self._bag_path = os.path.join(
            self._bag_save_folder,
            f'{datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}.bag',
        )
        command = ['rosbag', 'record']
        command += ['-O', self._bag_path]
        command += list(self._topic_handlers.keys())

        self._record_bag_process = subprocess.Popen(
            command,
        )
        self._recording = True
        self.started_recording_signal.emit()
        self.status_message_signal.emit(
            f'Started rosbag record with PID {self._record_bag_process.pid}\nRecording bag to {self._bag_path}'
        )

    def stop_recording(self):
        if not self._recording:
            msg = 'Cannot stop recording as not currently recording'
            self.status_message_signal.emit(msg)
            rospy.logwarn(msg)
            return
        self._record_bag_process.terminate()
        self._record_bag_process.wait()  # wait for the process to really terminate
        self._recording = False
        self.stopped_recording_signal.emit(self._bag_path)
        self.status_message_signal.emit(f'Finished recording bag to {self._bag_path}')

    def set_bag_save_folder(self, folder):
        self._bag_save_folder = folder
        self.set_bag_savedir_signal.emit(folder)

    def delete_bag(self):
        if self._recording:
            msg = 'Cannot delete the last bag while recording'
            self.status_message_signal.emit(msg)
            rospy.logwarn(msg)
            return
        if self._bag_path is None:
            msg = 'Cannot delete the last bag as it has not been recorded yet'
            self.status_message_signal.emit(msg)
            rospy.logwarn(msg)
            return
        os.remove(self._bag_path)
        self.status_message_signal.emit(f'Deleted bag at {self._bag_path}')
        self._bag_path = None

    def save_config(self, config_path):
        config = {
            'topics': list(self._topic_handlers.keys()),
            'bag_save_folder': self._bag_save_folder,
        }
        with open(config_path, 'w') as f:
            yaml.dump(config, f)

    def load_config(self, config_path):
        if self._recording:
            msg = 'Cannot load config while recording'
            self.status_message_signal.emit(msg)
            rospy.logwarn(msg)
            return

        with open(config_path, 'r') as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)

        self._bag_save_folder = config['bag_save_folder']
        self.set_bag_savedir_signal.emit(self._bag_save_folder)

        unavailable_topics = []
        loaded_topics = []
        for t in config['topics']:
            if t not in self._available_topics_types:
                unavailable_topics.append(t)
            else:
                self.add_topic(t)
                loaded_topics.append(t)
        self.check_topics_signal.emit(loaded_topics)

        msg = f'Loaded config from {config_path}'
        if len(unavailable_topics) > 0:
            msg += '\nCould not add the following topics as they are not currently published:'
            for t in unavailable_topics:
                msg += f'\n - {t}'
        self.status_message_signal.emit(msg)

    def close(self):
        if self._recording:
            self.stop_recording()
