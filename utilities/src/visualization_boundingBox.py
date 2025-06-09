#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_plugins.msg import ModelBoundingBoxes  
from gazebo_plugins.srv import retrieveBoundingBoxes, retrieveBoundingBoxesResponse
from geometry_msgs.msg import Point


class BBoxVisualizer:
    def __init__(self):
        rospy.init_node('bbox_visualizer_py')

        # Subscriber to Gazebo's bounding box topic
        rospy.Subscriber('/gazebo/bounding_boxes', ModelBoundingBoxes, self.bbox_callback)
        
        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        # Service for sending the bounding boxes information
        bb_serv = rospy.Service('retrieveBoundingBoxes', retrieveBoundingBoxes, self.handle_BoundingBoxes_request)
        
        rospy.loginfo("Bounding Box Visualizer (Python) started")
        rospy.spin()

    def handle_BoundingBoxes_request(self, req):
        return retrieveBoundingBoxesResponse(self.bounding_boxes_msg)

    def bbox_callback(self, msg):
        marker_array = MarkerArray()
        self.bounding_boxes_msg = msg
        
        for idx, (name, min_pt, max_pt) in enumerate(zip(msg.name, msg.min, msg.max)):
            marker = Marker()
            marker.header.frame_id = "map"  # Match Gazebo's frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = name  # Unique namespace per model
            marker.id = idx    # Unique ID
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.02  # Line thickness
            marker.color.r = 1.0    # Red color
            marker.color.a = 1.0    # Fully opaque
            
            # Calculate 8 corners of the bounding box
            corners = [
                Point(x=min_pt.x, y=min_pt.y, z=min_pt.z),  # 0
                Point(x=max_pt.x, y=min_pt.y, z=min_pt.z),  # 1
                Point(x=max_pt.x, y=max_pt.y, z=min_pt.z),  # 2
                Point(x=min_pt.x, y=max_pt.y, z=min_pt.z),  # 3
                Point(x=min_pt.x, y=min_pt.y, z=max_pt.z),  # 4
                Point(x=max_pt.x, y=min_pt.y, z=max_pt.z),  # 5
                Point(x=max_pt.x, y=max_pt.y, z=max_pt.z),  # 6
                Point(x=min_pt.x, y=max_pt.y, z=max_pt.z)   # 7
            ]
            
            # Define the 12 edges of the box
            edges = [
                (0,1), (1,2), (2,3), (3,0),  # Bottom face
                (4,5), (5,6), (6,7), (7,4),  # Top face
                (0,4), (1,5), (2,6), (3,7)   # Vertical edges
            ]
            
            # Add lines to the marker
            for start, end in edges:
                marker.points.append(corners[start])
                marker.points.append(corners[end])
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        BBoxVisualizer()
    except rospy.ROSInterruptException:
        pass