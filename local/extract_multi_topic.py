import rosbag
import cv2
import os, re
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import sys
# conda install libgcc
if __name__ == '__main__':

    bagname = sys.argv[1]

    home = bagname[:-4]
    rgb_dir = home + '/rgb'
    depth_dir = home + '/depth'
    if not os.path.exists(home):
        os.mkdir(home)
    if not os.path.exists(rgb_dir):
        os.mkdir(rgb_dir)
    if not os.path.exists(depth_dir):
        os.mkdir(depth_dir)

    bag = rosbag.Bag(bagname)

    t1 = '/yumi/ikSloverVel_controller/ee_cart_position'

    topics = bag.get_type_and_topic_info()[1].keys()
    t2 = '/camera/image/rgb_611205001943'
    t3 = '/camera/image/registered_depth__611205001943'
    bridge = CvBridge()

    print bag
    count = 0
    tt_pos = np.array([])
    for topic, msg, t in bag.read_messages(topics=[t1,t2,t3]):
        if topic == t1:
            count = count + 1
            print count
            pos = np.array(msg.data)
            tt_pos = np.vstack((tt_pos, pos)) if tt_pos.size else pos
        if topic == t2:
            cv_img = bridge.imgmsg_to_cv2(msg)
            cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)
            cv2.imwrite(os.path.join(rgb_dir + "/%04i.jpg" % count), cv_img)
        if topic == t3:
            cv_img = bridge.imgmsg_to_cv2(msg)
            cv2.imwrite(os.path.join(depth_dir + "/%04i.png" % count), cv_img)

    np.save(os.path.join(home + "/robot_pos"),tt_pos)
