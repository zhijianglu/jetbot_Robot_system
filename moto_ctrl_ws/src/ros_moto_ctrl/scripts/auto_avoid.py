#!/usr/bin/python3
# those for ctrl thread
import time
from threading import Thread
import rospy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray

# those for obstacle avoidance
import torch
import torchvision
import cv2
import numpy as np



import torch.nn.functional as F
import time
from threading import Thread

# those for image publisher
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


# todo program discribe:
# moving control
# in this program we will set multithread to do:
# 1. read and send imgs to host machine
# 2. read img and process avoidance and publish the prob of obstacle
# 3. subscrib to the topic of command and control the jetbot to move
# 4. to subscribe to left and right img to extimate the obstacle's direction 


# setup grobal params:
# ??
mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])
normalize = torchvision.transforms.Normalize(mean, stdev)

model = torchvision.models.alexnet(False)
model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, 2)
print('loading network, please wait ... ')
model.load_state_dict(torch.load('/home/lab/Project/py_test/best_model.pth'))
# model.load_state_dict(torch.load('/home/lab/Project/deep_learning/my_model1.pth'))
device = torch.device('cuda')
model = model.to(device)
print('finished load network ! ')
prob_blocked_l = 0.0
prob_blocked_r = 0.0
rospy.init_node("listener", anonymous=True)
pub_prob = rospy.Publisher('/prob_blocked', Float32MultiArray, queue_size=1)

# camera_obstacle = Camera.instance(width=224, height=224)
IMAGE_WIDTH=960
IMAGE_HEIGHT=720
# camera_obstacle = Camera.instance(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)

#those are obstacle avoidance functions:-----------------*
def preprocess(camera_value):
    global device, normalize

    x = camera_value
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x = x.transpose((2, 0, 1))
    x = torch.from_numpy(x).float()
    x = normalize(x)
    x = x.to(device)
    x = x[None, ...]

    return x

def update(change):
    global robot
    x = change['new']
    x = preprocess(x)
    y = model(x)

    # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
    y = F.softmax(y, dim=1)
    prob_blocked = float(y.flatten()[0])
    print('prob_blocked: %d', prob_blocked)

    if prob_blocked < 0.78:
        print('free to go', prob_blocked)
    else:
        print('hight risk, maybe blocked----------------')

    # ctrl command:
    # if prob_blocked < 0.5:
    #     robot.forward(0.4)
    # else:
    #     robot.left(0.4)
    # if prob_blocked < 0.78:
    #     robot.stop()
    #     robot.forward(0.7)
    # else:
    #     robot.stop()
    #     robot.left(0.7)

    time.sleep(0.001)

# those are python3 subscriber to node image, and process obstacle aovidence
def img_callback(imgmsg):
    global prob_blocked_l
    global prob_blocked_r

    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
    # frame = cv2.resize(frame, (224, 224))
    # cv2.imshow("listener", frame)
    # cv2.waitKey(1)
    # frame = cv2.resize(frame, (224, 224))

    x = preprocess(frame)
    y = model(x)

    # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
    y = F.softmax(y, dim=1)
    prob_blocked = float(y.flatten()[0])

    if imgmsg.header.stamp.to_sec() % 2 ==0:
        prob_blocked_l = prob_blocked
    else:
        prob_blocked_r = prob_blocked

    prob_blocked_msg = Float32MultiArray(data=[prob_blocked_l,prob_blocked_r])
    pub_prob.publish(prob_blocked_msg)

def main():
    print('listen to ctrl command topic')
    rospy.Subscriber("/camera/rgb/sub_img", Image, img_callback)
    rospy.spin()


if __name__ == '__main__':
    main()

