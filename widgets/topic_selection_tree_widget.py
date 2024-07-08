from PyQt5.Qt import Qt
from PyQt5.QtWidgets import (
    QTreeWidget,
    QTreeWidgetItem,
)
from PyQt5 import QtGui
from PyQt5.QtCore import pyqtSignal, QMutex


def format_num_display(num, decimals=2):
    if num < 10 ** (-decimals):
        return f'{num:.{decimals}e}'
    else:
        return f'{num:.{decimals}f}'


class TopicSelectionTreeWidget(QTreeWidget):
    checked_topic = pyqtSignal(str)
    unchecked_topic = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._groups_topics = {}

        self.itemChanged.connect(self._on_item_changed)
        self._topics_check_state = {}
        self._ignore_item_changed = False

        self.setColumnCount(2)
        self.setHeaderLabels(['Topics', 'Stats'])
        self.setColumnWidth(0, 600)

        self._mutex = QMutex()

    def _on_item_changed(self, item, column):
        if self._ignore_item_changed:
            return

        if item.childCount() > 0:
            # only care about changes for topics (leaf nodes)
            return

        if item.checkState(0) == self._topics_check_state[item.text(0)]:
            return
        else:
            self._topics_check_state[item.text(0)] = item.checkState(0)

        if item.checkState(0) == Qt.Checked:
            self.checked_topic.emit(item.text(0))
        else:
            self.unchecked_topic.emit(item.text(0))
            item.setText(1, '')  # clear displayed stats when unchecking a topic

    def add_topics(self, topics):
        self._mutex.lock()
        self._ignore_item_changed = True

        for t in topics:
            group = t.split('/')[1]
            if group not in self._groups_topics:
                self._groups_topics[group] = [t]
            else:
                if t in self._groups_topics[group]:
                    # do not add topic if it already exists
                    continue
                self._groups_topics[group].append(t)

            parent = None
            for i in range(self.topLevelItemCount()):
                if self.topLevelItem(i).text(0) == group:
                    parent = self.topLevelItem(i)
                    break

            if parent is None:
                parent = QTreeWidgetItem(self)
                parent.setText(0, group)
                parent.setFlags(
                    parent.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable
                )

            child = QTreeWidgetItem(parent)
            child.setFlags(child.flags() | Qt.ItemIsUserCheckable)
            child.setText(0, t)
            child.setText(1, '')
            child.setCheckState(0, Qt.Unchecked)
            self._topics_check_state[t] = Qt.Unchecked

        self.sortItems(0, Qt.AscendingOrder)

        self._ignore_item_changed = False
        self._mutex.unlock()

    def check_topics(self, topics):
        self._mutex.lock()
        self._ignore_item_changed = True

        for t in topics:
            if t not in self._topics_check_state:
                continue
            self._topics_check_state[t] = Qt.Checked
            for i in range(self.topLevelItemCount()):
                parent = self.topLevelItem(i)
                for j in range(parent.childCount()):
                    child = parent.child(j)
                    if child.text(0) == t:
                        child.setCheckState(0, Qt.Checked)
                        break

        self._ignore_item_changed = False
        self._mutex.unlock()

    def remove_topics(self, topics):
        for t in topics:
            group = t.split('/')[1]
            if group not in self._groups_topics:
                # cannot remove topic if its group does not exist
                continue

            if t not in self._groups_topics[group]:
                # cannot remove topic if it does not exist
                continue

            self._groups_topics[group].remove(t)
            if len(self._groups_topics[group]) == 0:
                del self._groups_topics[group]

            for i in range(self.topLevelItemCount()):
                parent = self.topLevelItem(i)
                if parent.text(0) == group:
                    for j in range(parent.childCount()):
                        child = parent.child(j)
                        if child.text(0) == t:
                            del self._topics_check_state[t]
                            parent.removeChild(child)
                            break
                    if parent.childCount() == 0:
                        self.invisibleRootItem().removeChild(parent)
                    break

    def freeze_selections(self):
        for i in range(self.topLevelItemCount()):
            parent = self.topLevelItem(i)
            parent.setDisabled(True)
            for j in range(parent.childCount()):
                child = parent.child(j)
                child.setDisabled(True)
                child.setForeground(1, QtGui.QColor('black'))

    def unfreeze_selections(self):
        for i in range(self.topLevelItemCount()):
            parent = self.topLevelItem(i)
            parent.setDisabled(False)
            for j in range(parent.childCount()):
                child = parent.child(j)
                child.setDisabled(False)

    def set_topics_stats(self, topics_stats):
        for i in range(self.topLevelItemCount()):
            parent = self.topLevelItem(i)
            for j in range(parent.childCount()):
                child = parent.child(j)
                child_topic = child.text(0)
                if child_topic in topics_stats:
                    stats_str = ''
                    stats_str += f'mean freq: {format_num_display(1 / topics_stats[child_topic]["mean"])}'
                    stats_str += f',    t since last [s]: {format_num_display(topics_stats[child_topic]["time_since_last_msg"])}'
                    stats_str += f',    std: {format_num_display(topics_stats[child_topic]["std"])}'
                    stats_str += f',    max: {format_num_display(topics_stats[child_topic]["max"])}'
                    stats_str += f',    min: {format_num_display(topics_stats[child_topic]["min"])}'

                    child.setText(1, stats_str)
