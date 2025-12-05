"""
ROS2 Logging Handler - Routes Python logging to ROS2 logger.

Based on: https://robotics.stackexchange.com/questions/114497/logging-from-non-ros-components-without-access-to-node
"""

import logging
from rclpy.logging import LoggingSeverity

# Mapping between ROS2 LoggingSeverity and Python logging levels
_ROS2_TO_PYTHON = {
    LoggingSeverity.DEBUG: logging.DEBUG,
    LoggingSeverity.INFO: logging.INFO,
    LoggingSeverity.WARN: logging.WARNING,
    LoggingSeverity.ERROR: logging.ERROR,
    LoggingSeverity.FATAL: logging.CRITICAL,
}
_PYTHON_TO_ROS2 = {v: k for k, v in _ROS2_TO_PYTHON.items()}


class ROS2LoggingHandler(logging.Handler):
    """Forwards Python log records to a ROS2 node's logger."""

    def __init__(self, node, level=logging.NOTSET):
        super().__init__(level)
        self._node_logger = node.get_logger()
        self.setFormatter(logging.Formatter("[%(name)s] %(message)s"))

    def emit(self, record):
        msg = self.format(record)
        ros2_level = _PYTHON_TO_ROS2.get(record.levelno)
        if ros2_level is None:
            # Find closest match for non-standard levels
            for py_level in sorted(_PYTHON_TO_ROS2.keys(), reverse=True):
                if record.levelno >= py_level:
                    ros2_level = _PYTHON_TO_ROS2[py_level]
                    break
            else:
                ros2_level = LoggingSeverity.DEBUG

        if ros2_level == LoggingSeverity.FATAL:
            self._node_logger.fatal(msg)
        elif ros2_level == LoggingSeverity.ERROR:
            self._node_logger.error(msg)
        elif ros2_level == LoggingSeverity.WARN:
            self._node_logger.warning(msg)
        elif ros2_level == LoggingSeverity.INFO:
            self._node_logger.info(msg)
        else:
            self._node_logger.debug(msg)


def setup_ros2_logging(node):
    """Set up Python logging to route through ROS2's logger."""
    ros2_level = node.get_logger().get_effective_level()
    level = _ROS2_TO_PYTHON.get(ros2_level, logging.DEBUG)

    handler = ROS2LoggingHandler(node, level)

    root_logger = logging.getLogger()
    for existing_handler in root_logger.handlers[:]:
        if isinstance(existing_handler, logging.StreamHandler):
            root_logger.removeHandler(existing_handler)

    root_logger.addHandler(handler)
    root_logger.setLevel(level)
    return handler


def cleanup_ros2_logging(handler):
    """Remove the ROS2 logging handler from the root logger."""
    root_logger = logging.getLogger()
    if handler in root_logger.handlers:
        root_logger.removeHandler(handler)
