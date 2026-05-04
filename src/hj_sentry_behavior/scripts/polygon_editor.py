#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Point, PointStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)


@dataclass
class Vertex:
    x: float
    y: float


PALETTE = [
    (0.93, 0.35, 0.23),  # red-orange
    (0.20, 0.62, 0.98),  # blue
    (0.18, 0.76, 0.43),  # green
    (0.95, 0.77, 0.06),  # yellow
    (0.61, 0.35, 0.71),  # violet
]


class PolygonEditor(Node):
    def __init__(self) -> None:
        super().__init__("polygon_editor")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("polygon_name", "polygon")
        self.declare_parameter("initial_polygon", "")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("marker_topic", "~/markers")
        self.declare_parameter("precision", 3)
        self.declare_parameter("close_polygon", True)
        self.declare_parameter("handle_scale", 0.45)
        self.declare_parameter("polygon_names", [self.get_parameter("polygon_name").value])
        self.declare_parameter(
            "polygon_strings", [self.get_parameter("initial_polygon").value]
        )
        self.declare_parameter("active_polygon", self.get_parameter("polygon_name").value)

        self.frame_id = self.get_parameter("frame_id").value
        self.polygon_name = self.get_parameter("polygon_name").value
        self.precision = int(self.get_parameter("precision").value)
        self.close_polygon = bool(self.get_parameter("close_polygon").value)
        self.handle_scale = float(self.get_parameter("handle_scale").value)
        self.polygon_names, self.polygon_vertices = self._load_polygons()
        self.active_polygon = self._resolve_active_polygon(
            self.get_parameter("active_polygon").value
        )
        self.color_map = {
            name: PALETTE[index % len(PALETTE)]
            for index, name in enumerate(self.polygon_names)
        }
        self.add_on_set_parameters_callback(self._set_parameters_callback)

        marker_topic = self.get_parameter("marker_topic").value
        clicked_point_topic = self.get_parameter("clicked_point_topic").value

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, clicked_point_topic, self._clicked_point_callback, 10
        )
        self.marker_server = InteractiveMarkerServer(self, "~/interactive")

        self.clear_service = self.create_service(
            Trigger, "~/clear", self._handle_clear_service
        )
        self.undo_service = self.create_service(
            Trigger, "~/undo_last", self._handle_undo_service
        )
        self.export_service = self.create_service(
            Trigger, "~/print_yaml", self._handle_export_service
        )

        self._refresh_visualization(log_yaml=self._has_any_vertices())

        self.get_logger().info(
            f"Polygon editor ready. Click in RViz on {clicked_point_topic} "
            "to add points, drag handles to fine tune."
        )
        self.get_logger().info(
            f"Services: {self.clear_service.srv_name}, "
            f"{self.undo_service.srv_name}, {self.export_service.srv_name}"
        )
        self.get_logger().info(
            f"Loaded polygons: {', '.join(self.polygon_names)}. "
            f"Active polygon: {self.active_polygon}"
        )
        self.get_logger().info(
            "Colors: " + ", ".join(
                f"{name}={self._color_label(name)}" for name in self.polygon_names
            )
        )
        self.get_logger().info(
            "Switch active polygon with: "
            "ros2 param set /polygon_editor active_polygon <polygon_name>"
        )

    def destroy_node(self) -> bool:
        self.marker_server.shutdown()
        return super().destroy_node()

    def _clicked_point_callback(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f"Clicked point frame '{msg.header.frame_id}' does not match configured "
                f"frame '{self.frame_id}'. Keeping coordinates as-is."
            )

        self.polygon_vertices[self.active_polygon].append(
            Vertex(x=msg.point.x, y=msg.point.y)
        )
        current_count = len(self.polygon_vertices[self.active_polygon]) - 1
        self.get_logger().info(
            f"Added vertex {self.active_polygon}:P{current_count} "
            f"at ({msg.point.x:.3f}, {msg.point.y:.3f})"
        )
        self._refresh_visualization(log_yaml=True)

    def _handle_clear_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.polygon_vertices[self.active_polygon].clear()
        self._refresh_visualization(log_yaml=True)
        response.success = True
        response.message = f"Cleared polygon '{self.active_polygon}'."
        return response

    def _handle_undo_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        vertices = self.polygon_vertices[self.active_polygon]
        if not vertices:
            response.success = False
            response.message = f"No vertices to undo in '{self.active_polygon}'."
            return response

        removed = vertices.pop()
        self._refresh_visualization(log_yaml=True)
        response.success = True
        response.message = (
            f"Removed last vertex from '{self.active_polygon}' "
            f"({removed.x:.3f}, {removed.y:.3f})."
        )
        return response

    def _handle_export_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        yaml_line = self._yaml_block()
        self.get_logger().info(f"Current polygons:\n{yaml_line}")
        response.success = True
        response.message = yaml_line
        return response

    def _interactive_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if not feedback.marker_name.startswith("vertex_"):
            return

        try:
            _, poly_index_str, vertex_index_str = feedback.marker_name.split("_", maxsplit=2)
            poly_index = int(poly_index_str)
            vertex_index = int(vertex_index_str)
        except (IndexError, ValueError):
            return

        if poly_index >= len(self.polygon_names):
            return

        polygon_name = self.polygon_names[poly_index]
        vertices = self.polygon_vertices[polygon_name]
        if vertex_index >= len(vertices):
            return

        vertices[vertex_index] = Vertex(
            x=feedback.pose.position.x,
            y=feedback.pose.position.y,
        )

        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            self._publish_display_markers()

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.get_logger().info(
                f"Moved vertex {polygon_name}:P{vertex_index} to "
                f"({feedback.pose.position.x:.3f}, {feedback.pose.position.y:.3f})"
            )
            self.get_logger().info(f"Updated polygons:\n{self._yaml_block()}")

    def _refresh_visualization(self, *, log_yaml: bool) -> None:
        self._rebuild_interactive_markers()
        self._publish_display_markers()
        if log_yaml:
            self.get_logger().info(f"Polygon YAML:\n{self._yaml_block()}")

    def _rebuild_interactive_markers(self) -> None:
        self.marker_server.clear()

        for poly_index, polygon_name in enumerate(self.polygon_names):
            for vertex_index, vertex in enumerate(self.polygon_vertices[polygon_name]):
                marker = InteractiveMarker()
                marker.header.frame_id = self.frame_id
                marker.name = f"vertex_{poly_index}_{vertex_index}"
                marker.description = f"{polygon_name}:P{vertex_index}"
                marker.scale = self.handle_scale * (
                    1.15 if polygon_name == self.active_polygon else 1.0
                )
                marker.pose.position.x = vertex.x
                marker.pose.position.y = vertex.y
                marker.pose.position.z = 0.05
                marker.pose.orientation.w = 1.0

                display_control = InteractiveMarkerControl()
                display_control.name = "display"
                display_control.always_visible = True
                display_control.interaction_mode = InteractiveMarkerControl.NONE
                display_control.markers.append(
                    self._make_handle_marker(polygon_name, vertex_index)
                )
                marker.controls.append(display_control)

                move_xy = InteractiveMarkerControl()
                move_xy.name = "move_xy"
                move_xy.orientation.w = 1.0
                move_xy.orientation.x = 0.0
                move_xy.orientation.y = 1.0
                move_xy.orientation.z = 0.0
                move_xy.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
                marker.controls.append(move_xy)

                move_x = InteractiveMarkerControl()
                move_x.name = "move_x"
                move_x.orientation.w = 1.0
                move_x.orientation.x = 1.0
                move_x.orientation.y = 0.0
                move_x.orientation.z = 0.0
                move_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                marker.controls.append(move_x)

                move_y = InteractiveMarkerControl()
                move_y.name = "move_y"
                move_y.orientation.w = 1.0
                move_y.orientation.x = 0.0
                move_y.orientation.y = 0.0
                move_y.orientation.z = 1.0
                move_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                marker.controls.append(move_y)

                self.marker_server.insert(
                    marker, feedback_callback=self._interactive_feedback
                )

        self.marker_server.applyChanges()

    def _publish_display_markers(self) -> None:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        stamp = self.get_clock().now().to_msg()

        for poly_index, polygon_name in enumerate(self.polygon_names):
            vertices = self.polygon_vertices[polygon_name]
            color = self.color_map[polygon_name]
            alpha = 1.0 if polygon_name == self.active_polygon else 0.7
            base_id = poly_index * 10000

            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = stamp
            line_marker.ns = "polygon"
            line_marker.id = base_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.10 if polygon_name == self.active_polygon else 0.07
            line_marker.color.r = color[0]
            line_marker.color.g = color[1]
            line_marker.color.b = color[2]
            line_marker.color.a = alpha
            line_marker.points = [self._to_point(vertex, z=0.03) for vertex in vertices]
            if self.close_polygon and len(vertices) >= 3:
                line_marker.points.append(self._to_point(vertices[0], z=0.03))
            marker_array.markers.append(line_marker)

            point_marker = Marker()
            point_marker.header.frame_id = self.frame_id
            point_marker.header.stamp = stamp
            point_marker.ns = "vertices"
            point_marker.id = base_id + 1
            point_marker.type = Marker.SPHERE_LIST
            point_marker.action = Marker.ADD
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.22 if polygon_name == self.active_polygon else 0.16
            point_marker.scale.y = point_marker.scale.x
            point_marker.scale.z = point_marker.scale.x
            point_marker.color.r = color[0]
            point_marker.color.g = color[1]
            point_marker.color.b = color[2]
            point_marker.color.a = 0.95
            point_marker.points = [self._to_point(vertex, z=0.05) for vertex in vertices]
            marker_array.markers.append(point_marker)

            title = Marker()
            title.header.frame_id = self.frame_id
            title.header.stamp = stamp
            title.ns = "titles"
            title.id = base_id + 2
            title.type = Marker.TEXT_VIEW_FACING
            title.action = Marker.ADD
            title.pose.position = self._title_point(vertices)
            title.pose.orientation.w = 1.0
            title.scale.z = 0.35
            title.color.r = color[0]
            title.color.g = color[1]
            title.color.b = color[2]
            title.color.a = 1.0
            title.text = polygon_name + (" [active]" if polygon_name == self.active_polygon else "")
            marker_array.markers.append(title)

            for vertex_index, vertex in enumerate(vertices):
                label = Marker()
                label.header.frame_id = self.frame_id
                label.header.stamp = stamp
                label.ns = "labels"
                label.id = base_id + 100 + vertex_index
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position = self._to_point(vertex, z=0.35)
                label.pose.orientation.w = 1.0
                label.scale.z = 0.28
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 1.0
                label.text = f"P{vertex_index}"
                marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)

    def _make_handle_marker(self, polygon_name: str, index: int) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        size = 0.22 if polygon_name == self.active_polygon else 0.16
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        color = self.color_map[polygon_name]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.95
        marker.pose.orientation.w = 1.0
        marker.text = f"P{index}"
        return marker

    def _yaml_line(self, polygon_name: str) -> str:
        return (
            f'{polygon_name}: '
            f'"{self._format_polygon_string(self.polygon_vertices[polygon_name])}"'
        )

    def _yaml_block(self) -> str:
        return "\n".join(self._yaml_line(name) for name in self.polygon_names)

    def _format_polygon_string(self, vertices: List[Vertex]) -> str:
        return ";".join(
            f"{vertex.x:.{self.precision}f},{vertex.y:.{self.precision}f}"
            for vertex in vertices
        )

    def _parse_polygon_string(self, polygon: str) -> List[Vertex]:
        vertices: List[Vertex] = []
        polygon = polygon.strip()
        if not polygon:
            return vertices

        for token in polygon.split(";"):
            token = token.strip()
            if not token:
                continue
            try:
                x_str, y_str = token.split(",", maxsplit=1)
                vertices.append(Vertex(x=float(x_str), y=float(y_str)))
            except ValueError as exc:
                raise ValueError(
                    f"Invalid polygon token '{token}', expected 'x,y'"
                ) from exc
        return vertices

    def _load_polygons(self) -> tuple[List[str], Dict[str, List[Vertex]]]:
        polygon_names = list(self.get_parameter("polygon_names").value)
        polygon_strings = list(self.get_parameter("polygon_strings").value)

        if len(polygon_strings) < len(polygon_names):
            polygon_strings.extend([""] * (len(polygon_names) - len(polygon_strings)))
        elif len(polygon_strings) > len(polygon_names):
            self.get_logger().warn(
                "polygon_strings has more entries than polygon_names. Extra entries will be ignored."
            )
            polygon_strings = polygon_strings[: len(polygon_names)]

        polygon_vertices = {
            name: self._parse_polygon_string(polygon_strings[index])
            for index, name in enumerate(polygon_names)
        }
        return polygon_names, polygon_vertices

    def _resolve_active_polygon(self, requested: str) -> str:
        if requested and requested in self.polygon_vertices:
            return requested
        return self.polygon_names[0]

    def _set_parameters_callback(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "active_polygon":
                continue
            requested = parameter.value
            if requested not in self.polygon_vertices:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"Unknown polygon '{requested}'. "
                        f"Expected one of: {', '.join(self.polygon_names)}"
                    ),
                )

        for parameter in parameters:
            if parameter.name == "active_polygon":
                self.active_polygon = parameter.value
                self.get_logger().info(
                    f"Active polygon switched to '{self.active_polygon}'"
                )
                self._refresh_visualization(log_yaml=False)

        return SetParametersResult(successful=True)

    def _has_any_vertices(self) -> bool:
        return any(self.polygon_vertices[name] for name in self.polygon_names)

    def _title_point(self, vertices: List[Vertex]) -> Point:
        if not vertices:
            return self._to_point(Vertex(0.0, 0.0), z=0.45)

        mean_x = sum(vertex.x for vertex in vertices) / len(vertices)
        mean_y = sum(vertex.y for vertex in vertices) / len(vertices)
        return self._to_point(Vertex(mean_x, mean_y), z=0.45)

    def _color_label(self, polygon_name: str) -> str:
        color = self.color_map[polygon_name]
        return f"rgb({int(color[0] * 255)},{int(color[1] * 255)},{int(color[2] * 255)})"

    @staticmethod
    def _to_point(vertex: Vertex, *, z: float) -> Point:
        point = Point()
        point.x = vertex.x
        point.y = vertex.y
        point.z = z
        return point


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolygonEditor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
