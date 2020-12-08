#!/usr/bin/env python
#coding:utf-8
import rospy, rospkg, rosparam
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from cv_bridge import CvBridge, CvBridgeError

class ArCodeRos():
    def __init__(self):
        print("\n######################")
        print("# Ar Marker Detector #")
        print("######################\n")

        rospy.init_node("arcode_ros", anonymous=False)
        
        self.sub_img_name = rospy.get_param("sub_img_name", "/camera/rgb/image_raw")
        self.show_result_img = rospy.get_param("show_result_img", False)
        self.show_result_console = rospy.get_param("show_result_console", False)

        self.img_subscriber = rospy.Subscriber(self.sub_img_name, Image, self.sub_img_callback)
        self.result_publisher = rospy.Publisher("/arcode_ros/detect_result", BoundingBoxArray, queue_size=10)

        self.aruco = cv2.aruco
        self.dic = self.aruco.getPredefinedDictionary(self.aruco.DICT_4X4_50) #生成したマーカーと合わせる
    
    def sub_img_callback(self, msg):
        try:
            cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            corners, ids, _ = self.aruco.detectMarkers(cv_img, self.dic)
            #https://qiita.com/marimori/items/74774685c7a2f1df0adc
            #
            # 0 - 1
            # |   |
            # 3 - 2

            if len(corners) == 0:
                if self.show_result_console == True:
                    rospy.logwarn("No AR Markers\n")
            
            else:
                bouning_box_array = BoundingBoxArray()

                for i in range(len(corners)):
                    bounding_box = BoundingBox()

                    center_x = (corners[i][0][2][0] + corners[i][0][0][0])/2
                    center_y = (corners[i][0][2][1] + corners[i][0][0][1])/2
                    bounding_box.pose.position.x = center_x
                    bounding_box.pose.position.y = center_y

                    radian = math.atan2(abs(corners[i][0][0][1]-corners[i][0][1][1]), abs(corners[i][0][0][0]-corners[i][0][1][0]))
                    bounding_box.pose.orientation.z = radian

                    dimension_x = corners[i][0][2][0] - corners[i][0][0][0]
                    dimension_y = corners[i][0][2][1] - corners[i][0][0][1]
                    bounding_box.dimensions.x = dimension_x
                    bounding_box.dimensions.y = dimension_y

                    bounding_box.header.frame_id = str(ids[i])
                    bounding_box.label = int(ids[i])

                    bouning_box_array.boxes.append(bounding_box)

                    if self.show_result_console == True:
                        rospy.loginfo("ID:%d (x:%d y:%d width:%d height:%d rad:%f)" % (ids[i], center_x, center_y, dimension_x, dimension_y, radian))

                        if i == len(corners)-1:
                            print("\n")

                self.result_publisher.publish(bouning_box_array)
    

            if self.show_result_img == True:
                self.aruco.drawDetectedMarkers(cv_img, corners, ids, (0, 255, 0))
                cv2.imshow("Ar Marker Result", cv_img)
                cv2.waitKey(1)

        except CvBridgeError, e:
            rospy.logerr(str(e))
            rospy.logerr("Failed to CvBridge.")

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    acr = ArCodeRos()
    acr.main()