#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from gazebo_plugins.srv import doorStringCommand



class Node():

    def __init__(self):

        rospy.init_node("pathNode",anonymous=True)
        self.scenario_number  = rospy.get_param("/scenario")


        self.initialPose_pub_turtlebot_1 = rospy.Publisher("/turtlebot3_1/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.initialPose_pub_turtlebot_2 = rospy.Publisher("/turtlebot3_2/initialpose", PoseWithCovarianceStamped, queue_size=10)

        self.client_initialPose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.clear_costmap_turtlebot_1 = rospy.ServiceProxy("/turtlebot3_1/move_base/clear_costmaps", Empty)
        self.clear_costmap_turtlebot_2 = rospy.ServiceProxy("/turtlebot3_2/move_base/clear_costmaps", Empty)


        self.rate = rospy.Rate(10) #10 Hz


if __name__=="__main__":

    pathNode = Node()

    rospy.sleep(3) 

    # SETTING THE INITIAL ESTIMATED POSE DURING LOCALIZATION
    response_list = [pathNode.client_initialPose("turtlebot_1",""), pathNode.client_initialPose("turtlebot_2","")]
    initialPose_publisher_list = [pathNode.initialPose_pub_turtlebot_1, pathNode.initialPose_pub_turtlebot_2]

    initialPose_msg = PoseWithCovarianceStamped()
    initialPose_msg.header.frame_id = "map"
    counter = 1

    for response, publisher in zip(response_list, initialPose_publisher_list):

        initialPose_msg.header.frame_id = f"turtlebot3_{counter}" + "/map"

        initialPose_msg.pose.pose.position.x = response.pose.position.x
        initialPose_msg.pose.pose.position.y = response.pose.position.y
        initialPose_msg.pose.pose.position.z = response.pose.position.z

        initialPose_msg.pose.pose.orientation.x = response.pose.orientation.x
        initialPose_msg.pose.pose.orientation.y = response.pose.orientation.y
        initialPose_msg.pose.pose.orientation.z = response.pose.orientation.z
        initialPose_msg.pose.pose.orientation.w = response.pose.orientation.w

        publisher.publish(initialPose_msg)
        counter += 1


    # OPEN ALL THE DOORS
    rospy.sleep(2)
    
    for i in range(2, 10):

        service_name = f"/door_{i}_control_service"
        
        rospy.wait_for_service(service_name)
        try:
            door_control = rospy.ServiceProxy(service_name, doorStringCommand)

            response = door_control("open")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed on %s: %s", service_name, e)


    # CLEAR COSTMAP
    rospy.sleep(2)
    pathNode.clear_costmap_turtlebot_1() 
    pathNode.clear_costmap_turtlebot_2() 
    
    rospy.spin() 