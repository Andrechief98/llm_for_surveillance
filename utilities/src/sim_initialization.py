#!/usr/bin/env python3
import rospy, rosservice, re, os, json
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from gazebo_plugins.srv import doorStringCommand


script_dir = os.path.dirname(__file__) 

class Node():

    def __init__(self):

        rospy.init_node("sim_initialization_node",anonymous=True)

        # Load the world configuration
        self.world_id = str(rospy.get_param('world_id', '1'))

        with open(f"{script_dir}/../../llm_interface/config/ROS/info_map{self.world_id}.json", "r") as f:
            info = json.load(f)

        # Retrieve robot, area and intruder information
        self.robots_dict = info["ros_publishers"]["robots"]
        self.robots_list = self.robots_dict.keys()

        # Initialize robot-specific publishers and clients in corresponding dictionaries
        self.initialPose_pubs_dict = {}
        self.clear_costmap_clients_dict = {}

        # Create initial pose publishers, costmap clients and goal subscriber for each robot
        for robot in self.robots_list:
            self.initialPose_pubs_dict[robot] = rospy.Publisher(f"/{robot}/initialpose", PoseWithCovarianceStamped, queue_size=10)
            self.clear_costmap_clients_dict[robot] = rospy.ServiceProxy(f"/{robot}/move_base/clear_costmaps",Empty)

        # Client to get gazebo model information
        self.client_initialPose = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.rate = rospy.Rate(10) #10 Hz



    def initialize_environment(self):

        # Delay to ensure Gazebo models are spawned
        rospy.sleep(8)

        ### INITIALIZE TURTLEBOT POSE    
        for robot in self.robots_list:

            try:
                # Retrieve and publish the initial pose of each robot
                response = self.client_initialPose(robot, "")

                if not response.success:
                    rospy.logerr("Failed to retrieve the pose of %s: %s", robot, response.status_message)
                    return False

                initialPose_msg = PoseWithCovarianceStamped()
                initialPose_msg.header.stamp = rospy.Time.now()
                initialPose_msg.header.frame_id = "map"

                initialPose_msg.pose.pose.position.x = response.pose.position.x
                initialPose_msg.pose.pose.position.y = response.pose.position.y
                initialPose_msg.pose.pose.position.z = response.pose.position.z

                initialPose_msg.pose.pose.orientation.x = response.pose.orientation.x
                initialPose_msg.pose.pose.orientation.y = response.pose.orientation.y
                initialPose_msg.pose.pose.orientation.z = response.pose.orientation.z
                initialPose_msg.pose.pose.orientation.w = response.pose.orientation.w

                self.initialPose_pubs_dict[robot].publish(initialPose_msg)

                rospy.loginfo("Initial pose published for %s", robot)

            except rospy.ServiceException as e:
                rospy.logerr(
                    "Failed to retrieve the pose of %s: %s",
                    robot,
                    e
                )
                return False


        ### CLEAR TURTLEBOT COSTMAPS  
        rospy.sleep(2)
        for robot, clear_costmap_client in self.clear_costmap_clients_dict.items():
            try:
                clear_costmap_client()
                rospy.loginfo("Costmap cleared for %s", robot)

            except rospy.ServiceException as e:
                rospy.logerr("Failed to clear the costmap for %s: %s", robot, e)


        ### OPEN ALL DOORS
        rospy.sleep(1)   
        active_services = rosservice.get_service_list()
        pattern = re.compile(r"^/door_(\d+)_control_service$")

        for service_name in active_services:
            if pattern.match(service_name):
                try:
                    rospy.wait_for_service(service_name)
                    door_control = rospy.ServiceProxy(service_name, doorStringCommand)
                    response = door_control("open")

                    rospy.loginfo("Service %s called successfully", service_name)

                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed on %s: %s", service_name, e)



if __name__=="__main__":

    sim_initialization_node = Node()

    sim_initialization_node.initialize_environment()

    # rospy.sleep(3)
    
    # # Wait for Gazebo service to be available (increased timeout)
    # try:
    #     rospy.loginfo("Waiting for /gazebo/get_model_state service...")
    #     rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    #     rospy.loginfo("/gazebo/get_model_state service is available")
    # except rospy.ROSException as e:
    #     rospy.logerr("Gazebo service not available: %s", e)
    #     rospy.logerr("Make sure Gazebo is running and properly initialized")
    #     rospy.signal_shutdown("Gazebo service unavailable")
    #     exit(1)

    # # Additional delay to ensure Gazebo models are spawned
    # rospy.sleep(5)

    # # SETTING THE INITIAL ESTIMATED POSE DURING LOCALIZATION
    # response_list = [pathNode.client_initialPose("turtlebot3_1",""), pathNode.client_initialPose("turtlebot3_2","")]
    # initialPose_publisher_list = [pathNode.initialPose_pub_turtlebot_1, pathNode.initialPose_pub_turtlebot_2]

    # initialPose_msg = PoseWithCovarianceStamped()
    # initialPose_msg.header.frame_id = "map"
    # counter = 1

    # for response, publisher in zip(response_list, initialPose_publisher_list):

    #     # initialPose_msg.header.frame_id = f"turtlebot3_{counter}" + "/map"
    #     initialPose_msg.header.frame_id = "map"

    #     initialPose_msg.pose.pose.position.x = response.pose.position.x
    #     initialPose_msg.pose.pose.position.y = response.pose.position.y
    #     initialPose_msg.pose.pose.position.z = response.pose.position.z

    #     initialPose_msg.pose.pose.orientation.x = response.pose.orientation.x
    #     initialPose_msg.pose.pose.orientation.y = response.pose.orientation.y
    #     initialPose_msg.pose.pose.orientation.z = response.pose.orientation.z
    #     initialPose_msg.pose.pose.orientation.w = response.pose.orientation.w

    #     publisher.publish(initialPose_msg)
    #     counter += 1


    # # Open all the doors in the environment (except door1 in the original world)
    # rospy.sleep(1)
    # world_id = str(rospy.get_param('world_id', '1'))

    # active_services = rosservice.get_service_list()
    # pattern = re.compile(r"^/door_(\d+)_control_service$")

    # for service_name in active_services:
    #     if pattern.match(service_name):
    #         try:
    #             rospy.wait_for_service(service_name)
    #             door_control = rospy.ServiceProxy(service_name, doorStringCommand)
    #             response = door_control("open")

    #             rospy.loginfo("Service %s called successfully", service_name)

    #         except rospy.ServiceException as e:
    #             rospy.logerr("Service call failed on %s: %s", service_name, e)


    # # Clear turtlebots' costmaps
    # rospy.sleep(2)
    # pathNode.clear_costmap_turtlebot_1() 
    # pathNode.clear_costmap_turtlebot_2() 
    
    rospy.spin() 