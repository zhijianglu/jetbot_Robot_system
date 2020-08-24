#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

rospy.init_node('listener', anonymous=True)
pub_prob = rospy.Publisher('/prob_blocked', Float32MultiArray, queue_size=1)


# those are python3 subscriber to node image
def callback(imgmsg):
    global pub_prob

    print (imgmsg.header.stamp.to_sec())

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(imgmsg)
    cv2.imshow("listener", img)
    cv2.waitKey(1)
    prob_blocked = 0.88
    data_pub = [0.88,0.99]
    pub_prob.publish(Float32MultiArray(data = data_pub))


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.Subscriber("/camera/rgb/sub_img", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
