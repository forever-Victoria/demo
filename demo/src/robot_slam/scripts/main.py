#!/usr/bin/env python2
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import sys
import os
import dynamic_reconfigure.client
from std_srvs.srv import Trigger, TriggerRequest
from abot_vlm.srv import LLMQuery
from TTS_audio.srv import TextToSpeech, TextToSpeechRequest
from TTS_audio.srv import StringService,StringServiceRequest
import random
import time
import json

#线索语音
calculate_result1_music="/home/abot/demo/src/robot_slam/mp3/grape.mp3"
calculate_result2_music="/home/abot/demo/src/robot_slam/mp3/5.mp3"
calculate_result3_music="/home/abot/demo/src/robot_slam/mp3/banana.mp3"
calculate_result4_music="/home/abot/demo/src/robot_slam/mp3/7.mp3"
calculate_result5_music="/home/abot/demo/src/robot_slam/mp3/apple.mp3"
calculate_result6_music="/home/abot/demo/src/robot_slam/mp3/8.mp3"
calculate_result7_music="/home/abot/demo/src/robot_slam/mp3/pear.mp3"
calculate_result8_music="/home/abot/demo/src/robot_slam/mp3/6.mp3"
#识别语音
music1_path="/home/abot/demo/src/robot_slam/img_detect/1.mp3"
music2_path="/home/abot/demo/src/robot_slam/img_detect/2.mp3"
music3_path="/home/abot/demo/src/robot_slam/img_detect/3.mp3"
music4_path="/home/abot/demo/src/robot_slam/img_detect/4.mp3"
#终点语音
music_end_path="/home/abot/demo/src/robot_slam/end_voice/1.mp3"
time =3
find_id = 0
id = 0
calculate_result = 0
result_received = False  # 新增标志位
identification = ""
ocr_text = ""  
clue=1
points=[
    [0,1,2],
    [3,4,5],
    [6,7,8],
    [9,10,11]
]
point_audio = {
    12: "/home/abot/demo/src/robot_slam/mp3/01.mp3",
    13: "/home/abot/demo/src/robot_slam/mp3/02.mp3",
    14: "/home/abot/demo/src/robot_slam/mp3/03.mp3",
    15: "/home/abot/demo/src/robot_slam/mp3/04.mp3"
}
class navigation_demo:
    def __init__(self):
        #rospy.init_node('navigation_demo', anonymous=True)
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        #self.current_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.current_pose_cb)
        #self.current_pose = None
        rospy.Subscriber('result', String, self.math_calculate)
        self.fruit_detection_service = rospy.ServiceProxy('/fruit_detection', Trigger)
        self.ocr_detection_service = rospy.ServiceProxy('/ocr_detection', Trigger)
        self.llm_query = rospy.ServiceProxy('llm_query', LLMQuery)
        self.text_to_speech = rospy.ServiceProxy('text_to_speech_service', TextToSpeech)
        self.tts_service = rospy.ServiceProxy('tts_service', StringService)    

    # def current_pose_cb(self, msg):
    #     self.current_pose = msg
    #     if self.current_pose is not None:
    #         self.set_pose_pub.publish(self.current_pose)

    def call_text_to_speech_service(self,text):
        rospy.wait_for_service('text_to_speech_service')
        try:
            request = TextToSpeechRequest()
            request.text = text  # 设置要传递的字符串
            response =self.text_to_speech(request)
            rospy.loginfo("Response:", response.success, response.message)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed:", e)

    def tts_client(self,text):
        rospy.wait_for_service('tts_service')
        try:
            request = StringServiceRequest(text)
            response = self.tts_service(request)
            rospy.loginfo("Response from server: %s", response.result)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed:", str(e))

    def llm_client(self,query):
        rospy.wait_for_service('llm_query')
        try:
            
            response = self.llm_query(query)
            return response.result
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def call_ocr_detection_service(self):
        # 等待服务可用
        rospy.loginfo("等待服务 /ocr_detection 可用...")
        rospy.wait_for_service('/ocr_detection')
        try:
            # 创建请求对象
            request = TriggerRequest()
            # 调用服务
            response = self.ocr_detection_service(request)
            # 打印服务响应
            rospy.loginfo("服务调用成功！识别结果: {}".format(response.message))

            return response.message
    
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: {}".format(e))

    def call_fruit_detection_service(self):
        # 等待服务可用
        rospy.loginfo("等待服务 /fruit_detection 可用...")
        rospy.wait_for_service('/fruit_detection')
        try:
            # 创建请求对象
            request = TriggerRequest()
            # 调用服务
            response = self.fruit_detection_service(request)
            # 打印服务响应
            rospy.loginfo("服务调用成功！识别结果: {}".format(response.message))
            return response.message
    
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: {}".format(e))
    def end24(self):
        global time
        msg = Twist()
        msg.linear.x = -0.3
        msg.linear.y = 0.3
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        while(time <= 13):
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time = time + 1
    def end13(self):
        global time
        msg = Twist()
        msg.linear.x = -0.3
        msg.linear.y = 0.3
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        while(time <= 13):
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time = time + 1
    def rotate(self):
        time1 = 0
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1
        while time1 <= 8:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1
    def right(self):
        time1 = 0
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = -0.5
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0
        while time1 <= 20:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1

    def ar_cb(self, data):
        global id
    # 若没有检测到任何AR标签，将id清零
        if len(data.markers) == 0:
            id = 0
            return
    # 有标签时，更新id
        for marker in data.markers:
            id = marker.id

    def math_calculate(self, msg):
        global calculate_result
        global result_received  # 使用全局标志位
        global identification
        rospy.loginfo("接收到结果: %s", msg.data)
        calculate_result = int(msg.data)
        if calculate_result == 1:
            identification ='葡萄'
            os.system('mplayer %s' % calculate_result1_music)
        elif calculate_result== 2:
            identification = '伍'
            os.system('mplayer %s' % calculate_result2_music)
        elif calculate_result == 3:
            identification = '香蕉'
            os.system('mplayer %s' % calculate_result3_music)
        elif calculate_result == 4:
            identification = 7
            os.system('mplayer %s' % calculate_result4_music)
        elif calculate_result == 5:
            identification = '苹果'
            os.system('mplayer %s' % calculate_result5_music)
        elif calculate_result == 6:
            identification = 8
            os.system('mplayer %s' % calculate_result6_music)
        elif calculate_result == 7:
            identification = '梨'
            os.system('mplayer %s' % calculate_result7_music)
        elif calculate_result == 8:
            identification = '陆'
            os.system('mplayer %s' % calculate_result8_music)

        result_received = True  # 设置标志位为True

    def find_cb(self, data):
        global find_id
        point_msg = data
        if (point_msg.z > 1 and point_msg.z <= 30) or (point_msg.z >= 241 and point_msg.z < 255) or (point_msg.z > 255 and point_msg.z <= 270):
            find_id = 1
        elif (point_msg.z >= 31 and point_msg.z <= 60) or (point_msg.z >= 271 and point_msg.z <= 300):
            find_id = 2
        elif (point_msg.z >= 61 and point_msg.z <= 90) or (point_msg.z >= 301 and point_msg.z <= 330):
            find_id = 3
        elif (point_msg.z >= 91 and point_msg.z <= 120) or (point_msg.z >= 331 and point_msg.z <= 360):
            find_id = 4
        elif (point_msg.z >= 121 and point_msg.z <= 150) or (point_msg.z >= 453 and point_msg.z <= 466):
            find_id = 5
        elif (point_msg.z >= 151 and point_msg.z <= 180) :
            find_id = 6
        elif (point_msg.z >= 181 and point_msg.z <= 210) :
            find_id = 7
        elif (point_msg.z >= 211 and point_msg.z <= 240) :
            find_id = 8

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))
        arrive_str = "arrived to target point"
        self.arrive_pub.publish(arrive_str)

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has been activated")

    def _feedback_cb(self, feedback):
        msg = feedback
        # rospy.loginfo("[Navi] navigation feedback\r\n%s" % feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % p)
        # arrive_str = "going to next point"
        # self.arrive_pub.publish(arrive_str)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
    

    def mission(self,point):
        global ocr_text, clue, id, find_id  
    # 核心新增：检测前主动清零，避免上一次残留数据
        id = 0
        find_id = 0
    # 等待100ms，确保清零操作生效（避免与回调函数竞争）
        rospy.sleep(0.1)
    
        self.goto(goals[point])
        rospy.set_param('/detect',1)     
        self.detect = self.call_fruit_detection_service() or ""  # 处理服务失败的空值
        rospy.loginfo("当前检测数据：detect=%s, find_id=%d, id=%d, identification=%s" 
                      % (self.detect, find_id, id, str(identification)))  # 新增日志，方便调试
    
    # 优化判断：只有当前检测的有效数据才参与匹配
        if (self.detect != "" and self.detect == identification) or \
           (find_id != 0 and find_id == identification) or \
           (id != 0 and id == identification):
            rospy.set_param('/ocr_det',1)
            ocr_detect = self.call_ocr_detection_service()
            rospy.loginfo("ocr识别结果: {}".format(ocr_detect))
        
            if ocr_detect is not None:
                ocr_text += ocr_detect
            else:
                print("OCR 识别失败，结果为 None")
        
        # 语音播放逻辑
            if clue==1:
                os.system('mplayer %s' % music1_path)
            elif clue==2:
                os.system('mplayer %s' % music2_path)
            elif clue==3:
                os.system('mplayer %s' % music3_path)
            elif clue==4:
                os.system('mplayer %s' % music4_path)
        
            navi.tts_client(ocr_detect)
            clue += 1  
    
    # 保留检测后重置，双重保障
        id = 0
        find_id = 0
  

    def recognize(self,p):
        global identification,id,find_id
        for i in range(3):
            self.mission(p[i])
        # 条件判断：同样添加空值排除
            if (self.detect != "" and self.detect == identification) or \
               (find_id != 0 and find_id == identification) or \
               (id != 0 and id == identification):

                return True



if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    local_obstacle_client = dynamic_reconfigure.client.Client("move_base/local_costmap/obstacle_layer")#初始化客户端
    local_obstacle_config = local_obstacle_client.get_configuration(timeout=8)#保存原始配置
    local_inf_client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer")
    local_inf_config = local_inf_client.get_configuration(timeout=8)
    global_static_client = dynamic_reconfigure.client.Client("move_base/global_costmap/static_layer")
    global_staitc_config = global_static_client.get_configuration(timeout=8)
    dwa_client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS")
    dwa_config = dwa_client.get_configuration(timeout=8)

    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    print('Please 1 to continue: ')
    input = raw_input()
    print(goals)
    r = rospy.Rate(1)
    navi = navigation_demo()
    if input == '1':
        # rospy.loginfo("setting dynamic parameters to avoid collision")
        # dwa_client.update_configuration({"max_vel_x": 4, "min_vel_x": -4, "max_vel_y": 4, "min_vel_y": -4, "max_vel_trans": 4, "min_vel_trans": -4})
        # local_obstacle_client.update_configuration({"enabled": 0})
        # global_static_client.update_configuration({"enabled": 0})
        # local_inf_client.update_configuration({"enabled": 0})

        # rospy.sleep(5)
        navi.goto(goals[0]) #移动识别计算题
        rospy.set_param('/im_flag', 1)
        while True:
            if result_received and calculate_result != 0:  # 检查标志位和calculate_result
                rospy.loginfo("calculate_result 有值，进行下一步操作")
                rospy.loginfo(identification)
                result_received = False  # 重置标志位
                break
        
        for i,p in enumerate(points):
            rospy.loginfo("开始识别第{}面墙:".format(i+1))
            navi.recognize(p)
        print(ocr_text)
        end_result = navi.llm_client(ocr_text)
        rospy.loginfo("LLM response: {}".format(end_result))
        navi.tts_client("最终答案是{}".format(end_result))
        #os.system('mplayer %s' % music_end_path)
        #navi.goto(goals[int(end_result)+11])#到达邻近点
        target_point = int(end_result) + 11
        if target_point not in [12, 13, 14, 15]:
              rospy.logwarn("目标点位超出范围（12-15），默认使用12号点")
              target_point = 12  # 超出范围时的默认值

# 移动到目标点位
        rospy.loginfo("前往目标点位：%d" % target_point)  # Python2 格式化语法
        navi.goto(goals[target_point])
        rospy.sleep(2)

# 播放对应点位的音频
        if target_point in point_audio:
            rospy.loginfo("播放点位%d的音频" % target_point)
            os.system('mplayer %s' % point_audio[target_point])
        else:
            rospy.logwarn("未找到点位%d对应的音频文件" % target_point)

# 根据点位执行对应函数
        if target_point in [12, 14]:
           navi.end13()  # 12、14点执行end13函数
        elif target_point in [13, 15]:
           navi.end24()  # 13、15点执行end24函数
        navi.goto(goals[16])#到达终点

        while not rospy.is_shutdown():
            r.sleep()
