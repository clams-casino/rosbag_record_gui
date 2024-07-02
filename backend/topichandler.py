import rospy
import roslib.message

import numpy as np


class TopicStats:
    def __init__(self, window_size=None):
        if window_size is None:
            # following implementation of rostopic
            # https://github.com/strawlab/ros_comm/blob/master/tools/rostopic/src/rostopic.py#L106
            self._window_size = 50000
        else:
            self._window_size = window_size

        self._last_msg_time = None
        self._diff_times = []

    def update(self, time):
        if self._last_msg_time is None:
            self._last_msg_time = time
            return

        self._diff_times.append(time - self._last_msg_time)
        if len(self._diff_times) > self._window_size:
            self._diff_times.pop(0)

        self._last_msg_time = time

    def get_stats(self):
        curr_time = rospy.get_time()
        if self._last_msg_time is None:
            time_since_last_msg = np.nan
        else:
            time_since_last_msg = curr_time - self._last_msg_time

        if len(self._diff_times) == 0:
            return {
                'mean': np.nan,
                'std': np.nan,
                'max': np.nan,
                'min': np.nan,
                'time_since_last_msg': time_since_last_msg,
            }

        diff_times = np.array(self._diff_times)
        return {
            'mean': np.mean(diff_times),
            'std': np.std(diff_times),
            'max': np.max(diff_times),
            'min': np.min(diff_times),
            'time_since_last_msg': time_since_last_msg,
        }


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
