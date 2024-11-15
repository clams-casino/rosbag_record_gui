import rospy
import roslib.message

import numpy as np


class TopicStats:
    def __init__(self, window_size=None):
        if window_size is None:
            # following implementation of rostopic
            # https://github.com/strawlab/ros_comm/blob/master/tools/rostopic/src/rostopic.py#L106
            self._diff_times_window_size = 50000
        else:
            self._diff_times_window_size = window_size

        self._last_msg_time = None
        self._diff_times_window = []
        self._times = None

    def update(self, time):
        if self._last_msg_time is None:
            self._last_msg_time = time
            return

        self._diff_times_window.append(time - self._last_msg_time)
        if len(self._diff_times_window) > self._diff_times_window_size:
            self._diff_times_window.pop(0)

        self._last_msg_time = time

        if self._times is not None:
            self._times.append(time)

    def get_stats(self):
        curr_time = rospy.get_time()
        if self._last_msg_time is None:
            time_since_last_msg = np.nan
        else:
            time_since_last_msg = curr_time - self._last_msg_time

        if len(self._diff_times_window) == 0:
            return {
                'mean': np.nan,
                'std': np.nan,
                'max': np.nan,
                'min': np.nan,
                'time_since_last_msg': time_since_last_msg,
            }

        diff_times = np.array(self._diff_times_window)
        return {
            'mean': np.mean(diff_times),
            'std': np.std(diff_times),
            'max': np.max(diff_times),
            'min': np.min(diff_times),
            'time_since_last_msg': time_since_last_msg,
        }

    def get_times(self):
        return self._times

    def start_recording_times(self):
        self._times = []

    def stop_recording_times(self):
        self._times = None


class TopicHandler:
    def __init__(self, topic_name, topic_type):
        self._stats = TopicStats()

        topic_class = roslib.message.get_message_class(topic_type)
        try:
            self._subscriber = rospy.Subscriber(topic_name, topic_class, self._callback)
        except Exception as e:
            rospy.logwarn(f'message class {topic_type} not built')
            self._subscriber = rospy.Subscriber(
                topic_name, rospy.AnyMsg, self._callback
            )

    def _callback(self, msg):
        curr_time = rospy.get_time()
        self._stats.update(curr_time)

    def get_stats(self):
        return self._stats.get_stats()

    def get_times(self):
        return self._stats.get_times()

    def start_recording_times(self):
        self._stats.start_recording_times()

    def stop_recording_times(self):
        self._stats.stop_recording_times()
