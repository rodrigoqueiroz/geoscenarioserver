"""
ROS message conversion and coordinate transformation utilities.

This module provides utilities for:
- Converting between local and WGS84 coordinate systems
- Converting between ROS messages and internal data structures
- Converting SimTraffic objects to ROS messages
"""

from typing import Optional, Dict, Any
from lanelet2.projection import LocalCartesianProjector
from lanelet2.core import BasicPoint3d, GPSPoint
from geoscenario_msgs.msg import Vehicle, Pedestrian


class CoordinateTransformer:
    """Handles coordinate transformations between local and WGS84 systems."""

    def __init__(self, use_wgs84: bool, projector: Optional[LocalCartesianProjector] = None):
        """
        Initialize the coordinate transformer.

        Args:
            use_wgs84: If True, use WGS84 coordinates; if False, use local coordinates
            projector: LocalCartesianProjector instance (required if use_wgs84=True)

        Raises:
            ValueError: If use_wgs84=True but projector is None
        """
        self.use_wgs84 = use_wgs84
        self.projector = projector

        if use_wgs84 and projector is None:
            raise ValueError("Projector required when use_wgs84=True")

    def agent_to_msg_position(self, agent_dict: Dict[str, float], msg) -> None:
        """
        Set ROS message position/velocity from agent dictionary.

        Args:
            agent_dict: Dictionary with keys: x, y, z, vx, vy
            msg: ROS message (Vehicle or Pedestrian) with position and velocity fields
        """
        if not self.use_wgs84:
            # Use local coordinates directly
            msg.position.x = agent_dict["x"]
            msg.position.y = agent_dict["y"]
            msg.position.z = agent_dict["z"]
            msg.velocity.x = agent_dict["vx"]
            msg.velocity.y = agent_dict["vy"]
        else:
            # Convert position to WGS84 coordinates
            wgs84_point = self.projector.reverse(
                BasicPoint3d(agent_dict["x"], agent_dict["y"], agent_dict["z"])
            )
            wgs84_vel_point = self.projector.reverse(
                BasicPoint3d(
                    agent_dict["x"] + agent_dict["vx"],
                    agent_dict["y"] + agent_dict["vy"],
                    agent_dict["z"]
                )
            )
            msg.position.x = wgs84_point.lat
            msg.position.y = wgs84_point.lon
            msg.position.z = wgs84_point.ele
            msg.velocity.x = wgs84_vel_point.lat - wgs84_point.lat
            msg.velocity.y = wgs84_vel_point.lon - wgs84_point.lon

    def msg_to_agent_position(self, msg, agent_dict: Dict[str, Any]) -> None:
        """
        Set agent dictionary position/velocity from ROS message.

        Args:
            msg: ROS message (Vehicle or Pedestrian) with position and velocity fields
            agent_dict: Dictionary to update with keys: x, y, z, vx, vy
        """
        if not self.use_wgs84:
            # Use local coordinates directly
            agent_dict["x"] = msg.position.x
            agent_dict["y"] = msg.position.y
            agent_dict["z"] = msg.position.z
            agent_dict["vx"] = msg.velocity.x
            agent_dict["vy"] = msg.velocity.y
        else:
            # Convert position from WGS84 to local coordinates
            local_point = self.projector.forward(
                GPSPoint(msg.position.x, msg.position.y, msg.position.z)
            )
            local_vel_point = self.projector.forward(
                GPSPoint(
                    msg.position.x + msg.velocity.x,
                    msg.position.y + msg.velocity.y,
                    msg.position.z
                )
            )
            agent_dict["x"] = local_point.x
            agent_dict["y"] = local_point.y
            agent_dict["z"] = local_point.z
            agent_dict["vx"] = local_vel_point.x - local_point.x
            agent_dict["vy"] = local_vel_point.y - local_point.y


class MessageConverter:
    """Converts between ROS messages and internal data structures."""

    def __init__(self, coordinate_transformer: CoordinateTransformer):
        """
        Initialize the message converter.

        Args:
            coordinate_transformer: CoordinateTransformer instance for handling coordinates
        """
        self.coord_transform = coordinate_transformer

    def vehicle_msg_to_dict(self, msg_vehicle) -> Dict[str, Any]:
        """
        Convert ROS Vehicle message to dictionary.

        Args:
            msg_vehicle: ROS Vehicle message

        Returns:
            Dictionary with vehicle properties
        """
        vehicle = {
            "id": msg_vehicle.id,
            "type": msg_vehicle.type,
            "l": msg_vehicle.dimensions.x,
            "w": msg_vehicle.dimensions.y,
            "h": msg_vehicle.dimensions.z,
            "yaw": msg_vehicle.yaw,
            "steering_angle": msg_vehicle.steering_angle,
            "active": msg_vehicle.active
        }
        self.coord_transform.msg_to_agent_position(msg_vehicle, vehicle)
        return vehicle

    def pedestrian_msg_to_dict(self, msg_pedestrian) -> Dict[str, Any]:
        """
        Convert ROS Pedestrian message to dictionary.

        Args:
            msg_pedestrian: ROS Pedestrian message

        Returns:
            Dictionary with pedestrian properties
        """
        pedestrian = {
            "id": msg_pedestrian.id,
            "type": msg_pedestrian.type,
            "l": msg_pedestrian.dimensions.x,
            "w": msg_pedestrian.dimensions.y,
            "h": msg_pedestrian.dimensions.z,
            "yaw": msg_pedestrian.yaw,
            "active": msg_pedestrian.active
        }
        self.coord_transform.msg_to_agent_position(msg_pedestrian, pedestrian)
        return pedestrian

    def dict_to_vehicle_msg(self, vehicle_dict: Dict[str, Any]) -> Vehicle:
        """
        Convert dictionary to ROS Vehicle message.

        Args:
            vehicle_dict: Dictionary with vehicle properties

        Returns:
            ROS Vehicle message
        """
        v_msg = Vehicle()
        v_msg.id = vehicle_dict["id"]
        v_msg.type = vehicle_dict["type"]
        v_msg.dimensions.x = vehicle_dict["l"]
        v_msg.dimensions.y = vehicle_dict["w"]
        v_msg.dimensions.z = vehicle_dict["h"]
        v_msg.yaw = vehicle_dict["yaw"]
        v_msg.steering_angle = vehicle_dict["steering_angle"]
        v_msg.active = True  # Not provided by shared memory, always True

        self.coord_transform.agent_to_msg_position(vehicle_dict, v_msg)
        return v_msg

    def dict_to_pedestrian_msg(self, pedestrian_dict: Dict[str, Any]) -> Pedestrian:
        """
        Convert dictionary to ROS Pedestrian message.

        Args:
            pedestrian_dict: Dictionary with pedestrian properties

        Returns:
            ROS Pedestrian message
        """
        p_msg = Pedestrian()
        p_msg.id = pedestrian_dict["id"]
        p_msg.type = pedestrian_dict["type"]
        p_msg.dimensions.x = pedestrian_dict["l"]
        p_msg.dimensions.y = pedestrian_dict["w"]
        p_msg.dimensions.z = pedestrian_dict["h"]
        p_msg.yaw = pedestrian_dict["yaw"]
        p_msg.active = True  # Not provided by shared memory, always True

        self.coord_transform.agent_to_msg_position(pedestrian_dict, p_msg)
        return p_msg
