#!/usr/bin/env python
#coding:utf-8
import rospy, rospkg
import cv2

PACKAGE_PATH = rospkg.RosPack().get_path("arcode_ros")
PIXEL_SIZE = 100

def ar_marker_generator():
    print("\n#######################")
    print("# AR Marker Generator #")
    print("#######################\n")
    
    aruco = cv2.aruco

    #定義済みの辞書を出力（確認用）
    #print(dir(aruco))

    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) #ブロックが4x4で0~50番までのマーカー

    for i in range(50):
        ar_marker_img = aruco.drawMarker(dic, i, PIXEL_SIZE) #ARマーカー画像を生成
        file_name = str(i).zfill(3) + ".png"
        cv2.imwrite(PACKAGE_PATH+"/marker_img/"+file_name, ar_marker_img)
        print("%s is saved @ %s" % (file_name, PACKAGE_PATH+"/marker_img/"))

if __name__ == "__main__":
    ar_marker_generator()