from __future__ import print_function

from cartographer_ros_msgs.srv import TrajectoryQuery
import sys
import rospy

'''
int32 trajectory_id
---
cartographer_ros_msgs/StatusResponse status
geometry_msgs/PoseStamped[] trajectory
'''

def query_client():
    rospy.wait_for_service('/trajectory_query')
    try:
        trajectory_query = rospy.ServiceProxy('/trajectory_query', TrajectoryQuery)
        resp1 = trajectory_query(0)
        print(resp1.status)
        f = open("cartographer_trajectory.txt","w")
        for pose in resp1.trajectory:
            result = "{}.{} {} {} {} {} {} {} {}\n".format(
                pose.header.stamp.secs, pose.header.stamp.nsecs,
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w
            )
            f.write(result)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    query_client()