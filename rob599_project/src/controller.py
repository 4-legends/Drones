#!/usr/bin/env python
import rospy 
from hector_uav_msgs.msg import PoseAction, PoseGoal
import  actionlib 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from rob599_project.srv import memorize_position, take_off, z_height, follow_moving
from rob599_project.msg import control_robotAction, control_robotGoal, control_robotFeedback, control_robotResult
from rob599_project.msg import patrolAction, patrolGoal, patrolResult, patrolFeedback
from rob599_project.msg import go_to_targetAction, go_to_targetGoal, go_to_targetFeedback, go_to_targetResult
from rob599_project.msg import go_to_markerAction, go_to_markerGoal, go_to_markerResult, go_to_markerFeedback


class Robot_controller:
    def __init__(self):
        self.known_location = {}
        self.heading = PoseStamped()
        self.z_height = 0.5
        rospy.Service('memorize_position', memorize_position, self.memorize_position_func)
        rospy.Service('take_off', take_off, self.take_off_func)
        rospy.Service('z_height', z_height, self.height_func)
        rospy.Service('follow_moving', follow_moving, self.follow_moving_func)
        self.server = actionlib.SimpleActionServer('control_robot', control_robotAction, self.action_callback, False)
        self.patrol_server = actionlib.SimpleActionServer('patrol', patrolAction, self.patrol_action_callback, False)
        self.server_go_to = actionlib.SimpleActionServer('go_to_target', go_to_targetAction, self.action_callback, False)
        self.go_to_target_server = actionlib.SimpleActionServer('go_to_marker', go_to_markerAction, self.go_to_marker_action_callback, False)
        self.server.start()
        self.go_to_target_server.start()
        self.server_go_to.start()
        self.patrol_server.start()
        self.marker_pub = rospy.Publisher('/marker',Marker, queue_size=1)
        self.moving_marker_pub = rospy.Publisher('/moving_marker',Marker, queue_size=1)
        self.pose_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.pose_callback)
        self.marker_loc = PoseStamped()
        rospy.sleep(1)
        fixed_marker = Marker()
        fixed_marker.action = Marker.ADD
        fixed_marker.type = Marker.CUBE
        fixed_marker.header.frame_id = 'world'
        fixed_marker.scale.x = 0.1
        fixed_marker.scale.y = 0.1
        fixed_marker.scale.z = 0.1
        fixed_marker.color.a = 1
        fixed_marker.color.r = 1.0
        fixed_marker.color.g = 0.0
        fixed_marker.color.b = 0.0    
        fixed_marker.pose.position.x = 1
        fixed_marker.pose.position.y = 2
        fixed_marker.pose.position.z = 0.5
        fixed_marker.pose.orientation.w = 1 
        self.marker_pub.publish(fixed_marker)
        

    def memorize_position_func(self, req):
        self.known_location[req.name] = self.heading
        return True


    def height_func(self, req):
        if req.height < 0 or req.height > 10:
            return False
        else:
            self.z_height = req.height
            return True
        
            
    def follow_moving_func(self, req):
        go_to_loc = PoseStamped()
        go_to_loc.header.frame_id = 'world'
        x = [-3, -2, -1, 0, 1, 2, 3]
        y = [-3, -2, -1, 0, 1, 2, 3]
        z = self.z_height
        for i in range(len(x)):
            moving_marker = Marker()
            moving_marker.action = Marker.ADD
            moving_marker.type = Marker.SPHERE
            moving_marker.header.frame_id = 'world'
            moving_marker.scale.x = 0.1
            moving_marker.scale.y = 0.1
            moving_marker.scale.z = 0.1
            moving_marker.color.a = 1
            moving_marker.color.r = 1.0
            moving_marker.color.g = 1.0
            moving_marker.color.b = 0.0    
            moving_marker.pose.position.x = x[i]
            moving_marker.pose.position.y = y[i]
            moving_marker.pose.position.z = z
            moving_marker.pose.orientation.w = 1 
            self.moving_marker_pub.publish(moving_marker)
            go_to_loc.pose.position.x = x[i]
            go_to_loc.pose.position.y = y[i]
            go_to_loc.pose.position.z = z
            client = actionlib.SimpleActionClient('action/pose', PoseAction)
            client.wait_for_server()
            rospy.loginfo('Made contact with Pose server')
            goal = PoseGoal()
            goal.target_pose = go_to_loc
            client.send_goal(goal)
            client.wait_for_result()
        return True


    def take_off_func(self, req):
        current_loc = self.heading
        current_loc.pose.position.z = self.z_height
        client = actionlib.SimpleActionClient('action/pose', PoseAction)
        client.wait_for_server()
        rospy.loginfo('Made contact with Pose server')
        goal = PoseGoal()
        goal.target_pose = current_loc
        client.send_goal(goal)
        client.wait_for_result()
        return True


    def pose_callback(self, msg):
        self.heading = msg
   

    def action_callback(self, goal_name):
        rospy.loginfo('Control Robot Action Server Started')
        try:
            goal_og = self.known_location[goal_name.name]
        except:
            rospy.logerr("Invalid name entered")
            return None
        client = actionlib.SimpleActionClient('action/pose', PoseAction)
        client.wait_for_server()
        rospy.loginfo('Made contact with Pose server')
        goal = PoseGoal()
        goal.target_pose = goal_og
        client.send_goal(goal)
        client.wait_for_result()
        result = 1
        self.server.set_succeeded(control_robotResult(status=result))
        rospy.loginfo("Control Robot Action Completed")  


    def patrol_action_callback(self, something):
        rospy.loginfo('Patrol Action Server Started')
        client = actionlib.SimpleActionClient('action/pose', PoseAction)
        client.wait_for_server()
        rospy.loginfo('Made contact with Pose server')
        for key, value in enumerate(self.known_location):
            goal_og = self.known_location[value]
            goal = PoseGoal()
            goal.target_pose = goal_og
            client.send_goal(goal)
            client.wait_for_result()
        result = 1
        self.patrol_server.set_succeeded(patrolResult(status=result))
        rospy.loginfo("Patrol Action Completed")


    def go_to_target_action_callback(self, goal_name):
        rospy.loginfo('go_to_target Action Server Started')
        goal_og = goal_name.goal
        if goal_og.header.frame_id != 'world':
            rospy.loginfo("Co-ordinated given in wrong frame")
            return 0
        client = actionlib.SimpleActionClient('action/pose', PoseAction)
        client.wait_for_server()
        rospy.loginfo('Made contact with Pose server')
        goal = PoseGoal()
        goal.target_pose = goal_og
        client.send_goal(goal)
        client.wait_for_result()
        result = 1
        self.server.set_succeeded(control_robotResult(status=result))
        rospy.loginfo("go_to_target Action Completed")

 
    def go_to_marker_action_callback(self, goal_name):
        rospy.loginfo('go_to_marker Action Server Started')
        goal_og = self.marker_loc
        client = actionlib.SimpleActionClient('action/pose', PoseAction)
        client.wait_for_server()
        rospy.loginfo('Made contact with Pose server')
        goal = PoseGoal()
        goal.target_pose = goal_og
        client.send_goal(goal)
        client.wait_for_result()
        result = 1
        self.server.set_succeeded(control_robotResult(status=result))
        rospy.loginfo("go_to_marker Action Completed")  



if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    controller = Robot_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
