#!/usr/bin/env python3

import rospy
from tm_msgs.msg import FeedbackState
from tm_msgs.srv import SetPositions, SetPositionsRequest
import math
import threading

robot_pose=[]


# current_point = [0.0] * 6
# control = True
targetP1 = [-0.12126, -0.59153, 0.28101, math.radians(-177.38), math.radians(59.36), math.radians(70.99)]
targetP2 = [0.13752, -0.826, 0.29728, math.radians(-177.38), math.radians(59.36), math.radians(70.99)]
targetP3 = [0.47982, -0.68626, 0.29728, math.radians(-177.38), math.radians(59.36), math.radians(70.99)]
# now_times = 0.0
# final_times = 3.0
rad_to_degree = 57.2958


def show():
    for i in range(100):
        print('test thread\n')


def monitor(monitor_target_point):
    global robot_pose
    # rospy.init_node('FeedbackState')
    rospy.init_node('demo_set_positions')
    sub = rospy.Subscriber('feedback_states', FeedbackState, tm_msg_callback, queue_size=1)
    rate = rospy.Rate(2)
    rospy.spin()
    # while not rospy.is_shutdown:
        
    #     rospy.spin()
    #     rate.sleep()
    #     change_of_point=abs(sum((monitor_target_point-robot_pose)))
    #     # if change_of_point<1:
    #     #     break

    """ while not rospy.is_shutdown():
        rospy.spinOnce()
        rate.sleep() """

    return 0

def monitor():
    global robot_pose
    # rospy.init_node('FeedbackState')
    rospy.init_node('demo_set_positions')
    sub = rospy.Subscriber('feedback_states', FeedbackState, tm_msg_callback, queue_size=1)
    rate = rospy.Rate(0.5)
    
   
    rate.sleep()
    rospy.spin()
    
        # change_of_point=abs(sum((monitor_target_point-robot_pose)))
        # if change_of_point<1:
        #     break

    """ while not rospy.is_shutdown():
        rospy.spinOnce()
        rate.sleep() """

    return 0

def tm_msg_callback(msg):
    global robot_pose
    if len(msg.tool_pose) == 6:
        """ rospy.loginfo("FeedbackState: joint pos = (%.2fdegree, %.2fdegree, %.2fdegree, %.2fdegree, %.2fdegree, %.2fdegree)",
                     msg.joint_pos[0]*rad_to_degree,
                     msg.joint_pos[1]*rad_to_degree, 
                     msg.joint_pos[2]*rad_to_degree,
                     msg.joint_pos[3]*rad_to_degree, 
                     msg.joint_pos[4]*rad_to_degree,
                     msg.joint_pos[5]*rad_to_degree) """
        rospy.loginfo("FeedbackState: tool pos = (%fm, %fm, %fm, %fdegree, %fdegree, %fdegree)",
                     msg.tool_pose[0],
                     msg.tool_pose[1], 
                     msg.tool_pose[2],
                     msg.tool_pose[3]*rad_to_degree, 
                     msg.tool_pose[4]*rad_to_degree,
                     msg.tool_pose[5]*rad_to_degree)


        robot_pose=msg.tool_pose[:]
        
        """ rospy.loginfo("tcp force -> FX=%.2f;FY=%.2f;FZ=%.2f",
                     msg.tcp_force[0], msg.tcp_force[1], msg.tcp_force[2]) """
    else:
        rospy.logerr("Error FeedbackState callback")




def go_to_position(Motion_type,target_point,velocity,accelerate_time,blend_percentage): #accelerate_time (sec)
    # Initialize ROS node
    global robot_pose
    rospy.init_node('demo_set_positions')

    # Create a service client for 'tm_driver/set_positions'
    rospy.wait_for_service('tm_driver/set_positions')
    client = rospy.ServiceProxy('tm_driver/set_positions', SetPositions)

    # Prepare the request
    req = SetPositionsRequest()
    req.motion_type = Motion_type #SetPositionsRequest.PTP_J

    # req.positions = [0, 0, 1.58, 0, 1.58, 0]
    # req.velocity = 0.4  # rad/s
    # req.acc_time = 0.2

    # rad_target_point=[x/rad_to_degree for x in target_point]
    # req.positions =rad_target_point

    req.positions =target_point


    req.velocity = velocity  # rad/s
    req.acc_time = accelerate_time
    
    req.blend_percentage = blend_percentage
    req.fine_goal = False

    # Call the service
    try:
        resp = client(req)
        if resp.ok:
            
            rospy.loginfo("SetPositions to robot")
            # monitor(target_point)
            # rospy.loginfo("current pose="+str(robot_pose))
        else:
            rospy.logwarn("SetPositions to robot, but response not yet ok")
    except rospy.ServiceException as e:
        rospy.logerr("Error SetPositions to robot: %s", e)
        return 1

    rospy.loginfo("Shutdown.")
    return 0





def main():
    # monitor(targetP1)
    go_to_position(SetPositionsRequest.PTP_T,targetP1,10,1,0) 
    # monitor(targetP2)
    go_to_position(SetPositionsRequest.PTP_T,targetP2,10,1,100)
    # monitor(targetP3)
    go_to_position(SetPositionsRequest.PTP_T,targetP3,10,1,0)

    return 0



if __name__ == '__main__':
    
    
    my_thread=threading.Thread(target=show)
    my_thread.start()
    # show()
    monitor()
    my_thread.join()
    
    # rospy.init_node('demo_set_positions')
    # sub = rospy.Subscriber('feedback_states', FeedbackState, tm_msg_callback, queue_size=10)
    # rate = rospy.Rate(2)
    
    # while not rospy.is_shutdown():
        
    #     rospy.spinOnce()
    #     rate = rospy.Rate(2)
        
    #     # if change_of_point<1:
    #     #     break

    """ while not rospy.is_shutdown():
        rospy.spinOnce()
        rate.sleep() """

    









