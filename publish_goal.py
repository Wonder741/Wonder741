#!/usr/bin/env python
import rospy
import math
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose

def publisher(p_trans):
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    if not rospy.is_shutdown():
        p = p_trans
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'map'
        pub.publish(p)
    return


if __name__ == '__main__':
    goal_x = [-2, 2, 0, -2, 1]
    goal_y = [-2, -4, 0, 4, 3]
    rospy.init_node('pose_goal', anonymous=True)
    rate = rospy.Rate(0.5) # Hz
    goal = PoseStamped()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    i = 0
    goal_flag = False
    try:
        while True:
            if not rospy.is_shutdown():
                try:
                    transformObject = tfBuffer.lookup_transform('odom', 'body_link', rospy.Time())
                    trans = transformObject.transform.translation
                    rot = transformObject.transform.rotation
                    goal_distance = abs(trans.x - goal_x[i]) + abs(trans.y - goal_y[i])
                    if goal_distance < 0.5 and not goal_flag:
                        i = i + 1
                        goal_flag = True
                        if i >= len(goal_x):
                            break
                    elif goal_distance > 0.5 and goal_flag:
                        goal_flag = False

                    goal.pose.position.x = goal_x[i]
                    goal.pose.position.y = goal_y[i]
                    goal.pose.orientation = rot
                    publisher(goal)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass

            rate.sleep()
    except rospy:
        pass
