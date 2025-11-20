#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
import rospy
import cv2
import numpy as np
from PIL import Image, ImageFont, ImageDraw
import time
from sensor_msgs.msg import Image as ROSImage
from std_srvs.srv import Trigger, TriggerResponse
from API_KEY import *
import json
import openai
from openai import OpenAI
import base64
import sys
import numpy as np

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

    # If the byte order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    # Convert to BGR if the encoding is not already BGR
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None

    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = ROSImage()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height  # That double line is actually integer division, not a comment
    return img_msg

def top_view_shot(image_msg):
    global detect
    '''
    这里接收来自话题/usb_cam/image_raw的ROS图像格式的消息，并保存图像，是否拍照用的参数服务器，然后设置参数就行，注意要加命名空间路径
    '''
    # 将ROS图像消息转换为OpenCV格式
    img_bgr = imgmsg_to_cv2(image_msg)
    # 从参数服务器获取detect的值
    detect = rospy.get_param('/detect', 255)
    
    if detect == 1:
        # 保存图像
        rospy.loginfo('保存至temp/vl_now.jpg')
        cv2.imwrite('/home/abot/demo/src/abot_vlm/temp2/vl_now.jpg', img_bgr)
        # 将detect重置为255
        rospy.set_param('/detect', 255)
        # # 屏幕上展示图像
        # cv2.imshow('vlm', img_bgr)
        cv2.waitKey(1)

        # 调用视觉大模型API
        yi_vision_api()
        #result_str = doubao_vision_api()
def yi_vision_api(PROMPT='图中有一种水果或者一个大写的数字，水果种类包含：香蕉、苹果、梨、葡萄这四种，大写数字包含：伍和陆，若存在这四种水果中的一种，请输出这个水果的名字；若存在这两种水果中的一种，请输出这个大写数字，若都不存在，请输出“无”', img_path='/home/abot/demo/src/abot_vlm/temp2/vl_now.jpg'):
    '''
    零一万物大模型开放平台，yi-vision视觉语言多模态大模型API
    '''
    
    client = OpenAI(
        api_key=YI_KEY,
        base_url="https://ark.cn-beijing.volces.com/api/v3"
    )
    
    # 编码为base64数据
    with open(img_path, 'rb') as image_file:
        image = 'data:image/jpeg;base64,' + base64.b64encode(image_file.read()).decode('utf-8')
    
    valid_results = ["香蕉", "苹果", "梨", "葡萄", "无","伍","陆"]
    
    while True:
        # 向大模型发起请求
        response = client.chat.completions.create(
            model="doubao-1-5-vision-pro-32k-250115",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": PROMPT},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": image
                            }
                        },
                    ]
                }
            ],
        )
        
        # 解析大模型返回结果
        result_str = response.choices[0].message.content.strip()
        result = str(result_str)
        
        rospy.loginfo('大模型调用成功！')
        rospy.loginfo(result)
        
        # 检查结果是否为指定的汉字
        if result in valid_results:
            return result
        else:
            print('结果不符合要求，重新识别...')

def handle_fruit_detection(req):
    # 调用视觉大模型API
    result = yi_vision_api()
    return TriggerResponse(success=True, message=result)

def main():
    global detect
    rospy.init_node('identify_node', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', ROSImage, top_view_shot)
    rospy.loginfo('视觉大模型模块导入成功！')
    rospy.loginfo('准备识别...')
    # 从参数服务器获取detect的值
    detect = rospy.set_param('/detect', 255)
    
    # 创建服务服务器
    s = rospy.Service('fruit_detection', Trigger, handle_fruit_detection)
    
    rospy.spin()

if __name__ == '__main__':
    main()
