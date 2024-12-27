#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

found_filtered = []
xml_path = "haarshare/haarcascade_upperbody.xml"
# 判断框中框
def is_inside(o, i):
    ox, oy, ow, oh = o
    ix, iy, iw, ih = i
    return ox > ix and oy > iy and ox + ow < ix + iw and oy + oh < iy + ih


# 框出人 传入图片 矩形参数 BGR
def draw_person(image, person, bgr):
    x, y, w, h = person
    cv2.rectangle(image, (x, y), (x + w, y + h), bgr, 2)


# 筛选识别出的人矩形数据
def screen_found(found):
    for ri, r in enumerate(found):
        for qi, q, in enumerate(found):
            if ri != qi and is_inside(r, q):
                break
            else:
                found_filtered.append(r)


def resize_keep_aspectratio(image_src, dst_size):
    src_h, src_w = image_src.shape[:2]
    # print(src_h, src_w)
    dst_h, dst_w = dst_size

    # 判断应该按哪个边做等比缩放
    h = dst_w * (float(src_h) / src_w)  # 按照ｗ做等比缩放
    w = dst_h * (float(src_w) / src_h)  # 按照h做等比缩放

    h = int(h)
    w = int(w)

    if h <= dst_h:
        image_dst = cv2.resize(image_src, (dst_w, int(h)))
    else:
        image_dst = cv2.resize(image_src, (int(w), dst_h))

    h_, w_ = image_dst.shape[:2]
    # print(h_, w_)
    print('等比缩放完毕')

    return image_dst


def cascade_video_person_detect_draw(video_path, xml_path, bgr, min_size, max_size):
    cap = cv2.VideoCapture(video_path)

    # 告诉OpenCV使用什么识别分类器
    classfier = cv2.CascadeClassifier(xml_path)

    while cap.isOpened():
        # 读取一帧数据
        ret, frame = cap.read()
        # 抓取不到视频帧，则退出循环
        if not ret:
            break
        # 显示方向
        frame = cv2.flip(frame, 1)

        # 将当前帧转换成灰度图像
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found = classfier.detectMultiScale(grey, scaleFactor=1.1, minNeighbors=4, minSize=min_size, maxSize=max_size)

        # 框出识别结果
        if len(found) > 0:
            for foundRect in found:
                x, y, w, h = foundRect
                cv2.rectangle(frame, (x - 10, y - 10), (x + w + 10, y + h + 10), bgr, 1)

        # 显示图像
        cv2.imshow('person detection', frame)
        # 键盘Q键结束
        c = cv2.waitKey(10)
        if c & 0xFF == ord('q'):
            break

    # 释放摄像头并销毁所有窗口
    cap.release()
    cv2.destroyAllWindows()

    print("视频识别结束")

class PersonTrackingNode:
    def __init__(self):
        rospy.init_node('person_tracking_node', anonymous=True)
        self.bridge = CvBridge()

        # 订阅摄像头话题
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/person_detection/image', Image, queue_size=1)
        self.person_info_pub = rospy.Publisher('/person_detection/info', String, queue_size=1)

        # 初始化级联分类器
        self.classifier = cv2.CascadeClassifier(xml_path)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 在这里执行人体跟踪和识别操作
        detected_image, person_info = self.detect_person(cv_image)

        # 发布识别结果图像
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(detected_image, 'bgr8'))

        # 发布识别的人物信息
        self.person_info_pub.publish(person_info)

    def detect_person(self, image):

        result_image, person_info = cascade_video_person_detect_draw(image, xml_path,(0, 255, 0), (50, 50), (1000, 3000)


        return result_image, person_info

if __name__ == '__main__':
    try:
        node = PersonTrackingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass