import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np

class TemplateMatcher:
def __init__(self):
self.bridge = CvBridge()
self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
self.image_pub = rospy.Publisher("/matched_image", Image, queue_size=10)
self.matched_number_pub = rospy.Publisher("/matched_number_position", Point, queue_size=10)

self.template = cv2.imread("3.png", cv2.IMREAD_GRAYSCALE) # 读取模板图像
self.template_height, self.template_width = self.template.shape

def image_callback(self, msg):
try:
cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # 将ROS图像消息转换成OpenCV格式
except Exception as e:
print(e)
return

result = cv2.matchTemplate(cv_image, self.template, cv2.TM_CCOEFF_NORMED)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

if max_val > 0.7: # 如果匹配度超过阈值
top_left = max_loc
bottom_right = (top_left[0] + self.template_width, top_left[1] + self.template_height)
cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)

matched_number_position = Point()
matched_number_position.x = (top_left[0] + bottom_right[0]) / 2
matched_number_position.y = (top_left[1] + bottom_right[1]) / 2
self.matched_number_pub.publish(matched_number_position)

matched_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8") # 将OpenCV格式的图像转换成ROS图像消息
self.image_pub.publish(matched_image_msg)

def main():
rospy.init_node('template_matching_node', anonymous=True)
template_matcher = TemplateMatcher()
rospy.spin()

if __name__ == '__main__':
main()
