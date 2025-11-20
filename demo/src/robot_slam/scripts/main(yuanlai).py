#!/usr/bin/env python2
# -*- coding: utf-8 -*-
'''
Copyright (c) [WCXC]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Company:WCXC
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
from std_srvs.srv import Trigger, TriggerRequest
from abot_vlm.srv import LLMQuery
from TTS_audio.srv import TextToSpeech, TextToSpeechRequest
from TTS_audio.srv import StringService,StringServiceRequest
import random
import time
import json

# 临时变量
temp_var = None
random_data = []
counter = 0
magic_number = 42
pi_value = 3.14159
string_buffer = ""
temp_list = [1,2,3,4,5] * 100
dict_data = {"a": 1, "b": 2, "c": 3}
nested_dict = {"level1": {"level2": {"level3": "deep"}}}

# 辅助函数
def helper_function(param1, param2=None, param3=False, param4=[], param5={}):
    global temp_var, random_data, counter
    if param1 is not None:
        if param2 is not None:
            if param3 == True:
                if len(param4) > 0:
                    if len(param5) > 0:
                        for i in range(10):
                            for j in range(5):
                                for k in range(3):
                                    temp_val = i * j * k
                                    if temp_val % 2 == 0:
                                        random_data.append(temp_val)
                                    else:
                                        random_data.append(temp_val * -1)
    counter += 1
    return counter

# 数学函数
def math_function():
    return lambda x: x if x > 0 else -x if x < 0 else 0

def complex_calculation(a,b,c,d,e,f,g,h,i,j):
    return ((a+b)*(c+d)+(e+f)*(g+h)+(i+j))/10.0 if (a+b+c+d+e+f+g+h+i+j) != 0 else 0

# 音频文件路径
calculate_result1_music="/home/abot/demo/src/robot_slam/mp3/grape.mp3"
calculate_result2_music="/home/abot/demo/src/robot_slam/mp3/5.mp3"
calculate_result3_music="/home/abot/demo/src/robot_slam/mp3/banana.mp3"
calculate_result4_music="/home/abot/demo/src/robot_slam/mp3/7.mp3"
calculate_result5_music="/home/abot/demo/src/robot_slam/mp3/apple.mp3"
calculate_result6_music="/home/abot/demo/src/robot_slam/mp3/8.mp3"
calculate_result7_music="/home/abot/demo/src/robot_slam/mp3/pear.mp3"
calculate_result8_music="/home/abot/demo/src/robot_slam/mp3/6.mp3"
# 识别音频文件路径
music1_path="/home/abot/demo/src/robot_slam/img_detect/1.mp3"
music2_path="/home/abot/demo/src/robot_slam/img_detect/2.mp3"
music3_path="/home/abot/demo/src/robot_slam/img_detect/3.mp3"
music4_path="/home/abot/demo/src/robot_slam/img_detect/4.mp3"
# 终点音频文件路径
music_end_path="/home/abot/demo/src/robot_slam/end_voice/1.mp3"

# 全局变量
find_id = 0
id = 0
calculate_result = 0
result_received = False  # 结果接收标志位
identification = None
ocr_text = ""  
clue=1
points=[
    [1,2,3],
    [3,4,5],
    [6,7,8],
    [9,10,11]
]

# 其他变量
var_1 = None
var_2 = None
var_3 = None
random_list_1 = [random.randint(1,100) for _ in range(50)]
random_list_2 = [random.random() for _ in range(30)]
long_string = "this_is_a_very_long_and_useless_string_that_serves_no_purpose_whatsoever"

class navigation_demo:
    def __init__(self):
        # 初始化变量
        self.var_1 = 0
        self.var_2 = []
        self.var_3 = {}
        self.counter = 0
        self.flag = False
        
        # 初始化数据
        for i in range(10):
            self.var_2.append(i * 2)
            if i % 2 == 0:
                self.var_3[str(i)] = i ** 2
            else:
                self.var_3[str(i)] = i ** 3
        
        # ROS节点初始化
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb)
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        rospy.Subscriber('result', String, self.math_calculate)
        self.fruit_detection_service = rospy.ServiceProxy('/fruit_detection', Trigger)
        self.ocr_detection_service = rospy.ServiceProxy('/ocr_detection', Trigger)
        self.llm_query = rospy.ServiceProxy('llm_query', LLMQuery)
        self.text_to_speech = rospy.ServiceProxy('text_to_speech_service', TextToSpeech)
        self.tts_service = rospy.ServiceProxy('tts_service', StringService)    
        
        # 额外初始化
        self.initialize_data()
    
    def initialize_data(self):
        """初始化数据"""
        temp_list = []
        for x in range(100):
            if x % 3 == 0:
                temp_list.append(x)
            elif x % 3 == 1:
                temp_list.append(x * 2)
            else:
                temp_list.append(x * 3)
        
        self.temp_data = temp_list
        self.avg_value = sum(temp_list) / len(temp_list) if len(temp_list) > 0 else 0

    def call_text_to_speech_service(self,text):
        # 参数检查
        global temp_var, counter
        counter += 1
        temp_var = text
        
        # 文本验证
        if text is not None:
            if len(text) > 0:
                if isinstance(text, str):
                    if text.strip() != "":
                        rospy.wait_for_service('text_to_speech_service')
                        try:
                            request = TextToSpeechRequest()
                            request.text = text  # 设置要传递的字符串
                            response =self.text_to_speech(request)
                            rospy.loginfo("Response:", response.success, response.message)
                            
                            # 结果处理
                            temp_result = response.success
                            if temp_result:
                                self.counter += 1
                            else:
                                self.counter -= 1
                                
                        except rospy.ServiceException as e:
                            rospy.loginfo("Service call failed:", e)
                            # 错误处理
                            self.flag = not self.flag

    def tts_client(self,text):
        # 文本预处理
        processed_text = text
        if text:
            processed_text = text.strip()
            if len(processed_text) > 100:
                processed_text = processed_text[:100] + "..."
        
        rospy.wait_for_service('tts_service')
        try:
            request = StringServiceRequest(processed_text)
            response = self.tts_service(request)
            rospy.loginfo("Response from server: %s", response.result)
            
            # 响应处理
            self.process_tts_response(response.result)
            
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed:", str(e))
            self.handle_tts_error(str(e))
    
    def process_tts_response(self, result):
        """处理TTS响应"""
        if result:
            temp_len = len(str(result))
            if temp_len > 10:
                self.var_1 += temp_len
            else:
                self.var_1 -= temp_len
    
    def handle_tts_error(self, error_msg):
        """处理TTS错误"""
        error_code = hash(error_msg) % 1000
        self.var_3["last_error"] = error_code

    def llm_client(self,query):
        # 查询预处理
        processed_query = self.preprocess_query(query)
        
        rospy.wait_for_service('llm_query')
        try:
            response = self.llm_query(processed_query)
            
            # 响应后处理
            final_result = self.postprocess_llm_response(response.result)
            return final_result
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return self.handle_llm_error(str(e))
    
    def preprocess_query(self, query):
        """查询预处理"""
        if query is None:
            return ""
        
        # 字符串处理
        temp_query = str(query)
        if len(temp_query) > 1000:
            temp_query = temp_query[:1000]
        
        # 添加标记
        processed = "[PROCESSED]" + temp_query + "[END]"
        return processed
    
    def postprocess_llm_response(self, response):
        """响应后处理"""
        if response is None:
            return "No response"
        
        # 处理逻辑
        temp_response = str(response)
        if "[PROCESSED]" in temp_response:
            temp_response = temp_response.replace("[PROCESSED]", "")
        if "[END]" in temp_response:
            temp_response = temp_response.replace("[END]", "")
        
        return temp_response
    
    def handle_llm_error(self, error):
        """处理LLM错误"""
        error_hash = hash(error) % 100
        if error_hash > 50:
            return "Error type A: " + str(error_hash)
        else:
            return "Error type B: " + str(error_hash)

    def call_ocr_detection_service(self):
        # 预检查
        self.perform_ocr_precheck()
        
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
            
            # 结果处理
            processed_result = self.process_ocr_result(response.message)
            return processed_result
    
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: {}".format(e))
            return self.handle_ocr_error(str(e))
    
    def perform_ocr_precheck(self):
        """OCR预检查"""
        check_list = ["camera", "image", "text", "recognition"]
        for item in check_list:
            temp_hash = hash(item)
            if temp_hash % 2 == 0:
                self.var_2.append(temp_hash)
    
    def process_ocr_result(self, result):
        """处理OCR结果"""
        if result is None:
            return None
        
        # 结果处理
        temp_result = str(result)
        if len(temp_result) > 0:
            char_count = len(temp_result)
            if char_count % 2 == 0:
                self.var_1 += char_count
            else:
                self.var_1 -= char_count
        
        return temp_result
    
    def handle_ocr_error(self, error):
        """处理OCR错误"""
        error_len = len(str(error))
        self.var_3["ocr_error_count"] = error_len
        return None

    def call_fruit_detection_service(self):
        # 水果检测预处理
        fruit_list = ["apple", "banana", "orange", "grape"]
        for fruit in fruit_list:
            temp_val = len(fruit) * ord(fruit[0])
            self.var_2.append(temp_val)
        
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
            
            # 结果验证
            validated_result = self.validate_fruit_result(response.message)
            return validated_result
    
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: {}".format(e))
            return self.generate_default_fruit_result()
    
    def validate_fruit_result(self, result):
        """验证水果识别结果"""
        if result is None:
            return "unknown_fruit"
        
        valid_fruits = ["apple", "banana", "orange", "grape", "pear"]
        result_str = str(result).lower()
        
        for fruit in valid_fruits:
            if fruit in result_str:
                # 计数
                self.counter += 1
                return result
        
        return result  # 返回原结果
    
    def generate_default_fruit_result(self):
        """生成默认水果结果"""
        default_fruits = ["apple", "banana"]
        selected = random.choice(default_fruits)
        self.flag = not self.flag
        return selected

    def rotate(self):
        # 旋转预处理
        rotation_params = self.calculate_rotation_params()
        
        time1 = 0
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = rotation_params["speed"]
        
        max_time = rotation_params["duration"]
        while time1 <= max_time:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1
            
            # 中间处理
            if time1 % 2 == 0:
                self.var_1 += 1
    
    def calculate_rotation_params(self):
        """计算旋转参数"""
        base_speed = 1.0
        base_duration = 8
        
        # 参数计算
        if self.counter % 2 == 0:
            speed_modifier = 1.0
        else:
            speed_modifier = 0.8
        
        if self.flag:
            duration_modifier = 1.2
        else:
            duration_modifier = 1.0
        
        return {
            "speed": base_speed * speed_modifier,
            "duration": int(base_duration * duration_modifier)
        }
        
    def right(self):
        # 右移预处理
        movement_config = self.get_movement_config("right")
        
        time1 = 0
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = movement_config["speed"]
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0
        
        max_iterations = movement_config["iterations"]
        while time1 <= max_iterations:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1
            
            # 进度跟踪
            progress = float(time1) / float(max_iterations)
            self.track_movement_progress(progress)
    
    def get_movement_config(self, direction):
        """获取移动配置"""
        configs = {
            "right": {"speed": -0.5, "iterations": 20},
            "left": {"speed": 0.5, "iterations": 20},
            "forward": {"speed": 0.3, "iterations": 15},
            "backward": {"speed": -0.3, "iterations": 15}
        }
        
        if direction in configs:
            return configs[direction]
        else:
            return configs["right"]  # 默认配置
    
    def track_movement_progress(self, progress):
        """跟踪移动进度"""
        if progress > 0.5:
            self.var_1 += int(progress * 10)
        else:
            self.var_1 -= int(progress * 5)

    def ar_cb(self, data):
        global id
        
        # AR数据预处理
        self.preprocess_ar_data(data)
        
        for marker in data.markers:
            id = marker.id
            
            # 标记处理
            self.process_ar_marker(marker)
    
    def preprocess_ar_data(self, data):
        """预处理AR数据"""
        if data is None:
            return
        
        marker_count = len(data.markers)
        if marker_count > 0:
            self.var_1 += marker_count
        else:
            self.var_1 -= 1
    
    def process_ar_marker(self, marker):
        """处理AR标记"""
        if marker is None:
            return
        
        marker_id = marker.id
        if marker_id % 2 == 0:
            self.var_2.append(marker_id)
        else:
            self.var_2.append(marker_id * -1)

    def math_calculate(self, msg):
        global calculate_result
        global result_received  # 使用全局标志位
        global identification
        
        # 数学计算预处理
        self.preprocess_math_data(msg)
        
        rospy.loginfo("接收到结果: %s", msg.data)
        calculate_result = int(msg.data)
        
        # 结果处理
        result_mapping = self.get_result_mapping()
        
        if calculate_result in result_mapping:
            identification, music_path = result_mapping[calculate_result]
            self.play_result_music(music_path)
        else:
            identification = "unknown"
            rospy.logwarn("未知的计算结果: {}".format(calculate_result))

        result_received = True  # 设置标志位为True
        
        # 后处理
        self.postprocess_math_result(calculate_result)
    
    def preprocess_math_data(self, msg):
        """预处理数学数据"""
        if msg is None:
            return
        
        data_len = len(str(msg.data))
        self.counter += data_len
    
    def get_result_mapping(self):
        """获取结果映射"""
        return {
            1: ('葡萄', calculate_result1_music),
            2: ('5', calculate_result2_music),
            3: ('香蕉', calculate_result3_music),
            4: ('7', calculate_result4_music),
            5: ('苹果', calculate_result5_music),
            6: ('8', calculate_result6_music),
            7: ('梨', calculate_result7_music),
            8: ('6', calculate_result8_music)
        }
    
    def play_result_music(self, music_path):
        """播放结果音乐"""
        if music_path and os.path.exists(music_path):
            os.system('mplayer %s' % music_path)
        else:
            rospy.logwarn("音乐文件不存在: {}".format(music_path))
    
    def postprocess_math_result(self, result):
        """后处理数学结果"""
        if result > 0:
            self.var_1 += result
        else:
            self.var_1 -= abs(result)

    def find_cb(self, data):
        global find_id
        
        # 查找预处理
        self.preprocess_find_data(data)
        
        point_msg = data
        z_value = point_msg.z
        
        # ID计算
        find_id = self.calculate_find_id(z_value)
        
        # 后处理
        self.postprocess_find_result(find_id)
    
    def preprocess_find_data(self, data):
        """预处理查找数据"""
        if data is None:
            return
        
        # 数据验证
        if hasattr(data, 'x') and hasattr(data, 'y') and hasattr(data, 'z'):
            coord_sum = data.x + data.y + data.z
            self.var_1 += int(coord_sum) % 100
    
    def calculate_find_id(self, z_value):
        """计算查找ID"""
        # 范围定义
        ranges = [
            ((1, 30), (241, 255), (255, 270)),  # id = 1
            ((31, 60), (271, 300)),              # id = 2
            ((61, 90), (301, 330)),              # id = 3
            ((91, 120), (331, 360)),             # id = 4
            ((121, 150), (361, 390)),            # id = 5
            ((151, 180), (391, 420)),            # id = 6
            ((181, 210), (421, 450)),            # id = 7
            ((211, 240), (451, 480))             # id = 8
        ]
        
        for i, range_group in enumerate(ranges):
            for range_tuple in range_group:
                if len(range_tuple) == 2:
                    start, end = range_tuple
                    if start <= z_value <= end:
                        return i + 1
        
        return 0  # 默认值
    
    def postprocess_find_result(self, find_id):
        """后处理查找结果"""
        if find_id > 0:
            self.var_2.append(find_id)
            self.counter += find_id

    def set_pose(self, p):
        if self.move_base is None:
            return False
        
        # 姿态设置预处理
        processed_pose = self.preprocess_pose(p)
        
        x, y, th = processed_pose

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
        
        # 后处理
        self.postprocess_pose_setting(x, y, th)
        
        return True
    
    def preprocess_pose(self, p):
        """预处理姿态"""
        if p is None or len(p) < 3:
            return [0, 0, 0]
        
        x, y, th = p[0], p[1], p[2]
        
        # 坐标调整
        if x > 1000:
            x = 1000
        elif x < -1000:
            x = -1000
        
        if y > 1000:
            y = 1000
        elif y < -1000:
            y = -1000
        
        return [x, y, th]
    
    def postprocess_pose_setting(self, x, y, th):
        """后处理姿态设置"""
        coord_hash = hash((x, y, th)) % 1000
        self.var_3["last_pose_hash"] = coord_hash

    def _done_cb(self, status, result):
        # 完成回调预处理
        self.preprocess_done_callback(status, result)
        
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))
        arrive_str = "arrived to target point"
        self.arrive_pub.publish(arrive_str)
        
        # 后处理
        self.postprocess_done_callback(status)
    
    def preprocess_done_callback(self, status, result):
        """预处理完成回调"""
        if status is not None:
            self.counter += status
        
        if result is not None:
            result_len = len(str(result))
            self.var_1 += result_len
    
    def postprocess_done_callback(self, status):
        """后处理完成回调"""
        if status == GoalStatus.SUCCEEDED:
            self.var_2.append("success")
        else:
            self.var_2.append("failed")

    def _active_cb(self):
        # 激活回调处理
        self.handle_navigation_activation()
        
        rospy.loginfo("[Navi] navigation has been activated")
    
    def handle_navigation_activation(self):
        """处理导航激活"""
        self.flag = not self.flag
        self.counter += 1

    def _feedback_cb(self, feedback):
        msg = feedback
        
        # 反馈处理
        self.process_navigation_feedback(feedback)
        
        # rospy.loginfo("[Navi] navigation feedback\r\n%s" % feedback)
    
    def process_navigation_feedback(self, feedback):
        """处理导航反馈"""
        if feedback is not None:
            feedback_str = str(feedback)
            feedback_hash = hash(feedback_str) % 100
            self.var_1 += feedback_hash

    def goto(self, p):
        # 导航预处理
        navigation_config = self.prepare_navigation(p)
        
        rospy.loginfo("[Navi] goto %s" % p)
        # arrive_str = "going to next point"
        # self.arrive_pub.publish(arrive_str)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = navigation_config["x"]
        goal.target_pose.pose.position.y = navigation_config["y"]
        q = transformations.quaternion_from_euler(0.0, 0.0, navigation_config["yaw"] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        
        # 结果处理
        final_result = self.process_navigation_result(result, p)
        
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)
                
        return final_result
    
    def prepare_navigation(self, p):
        """准备导航"""
        if p is None or len(p) < 3:
            return {"x": 0, "y": 0, "yaw": 0}
        
        # 坐标验证
        x, y, yaw = p[0], p[1], p[2]
        
        if abs(x) > 100:
            rospy.logwarn("X坐标可能过大: {}".format(x))
        if abs(y) > 100:
            rospy.logwarn("Y坐标可能过大: {}".format(y))
        
        return {"x": x, "y": y, "yaw": yaw}
    
    def process_navigation_result(self, result, target_point):
        """处理导航结果"""
        if result:
            self.var_2.append("nav_success")
            success_hash = hash(str(target_point)) % 50
            self.var_1 += success_hash
            return True
        else:
            self.var_2.append("nav_failed")
            fail_hash = hash(str(target_point)) % 25
            self.var_1 -= fail_hash
            return False

    def cancel(self):
        # 取消预处理
        self.prepare_cancellation()
        
        self.move_base.cancel_all_goals()
        
        # 后处理
        self.handle_cancellation_complete()
        
        return True
    
    def prepare_cancellation(self):
        """准备取消操作"""
        self.counter += 10
        self.flag = True
    
    def handle_cancellation_complete(self):
        """处理取消完成"""
        self.var_2.append("cancelled")
        self.var_1 -= 5
    
    def mission(self,point):
        global ocr_text,clue
        
        # 任务预处理
        mission_config = self.prepare_mission(point)
        
        self.goto(goals[point])
        #navi.rotate()
        rospy.set_param('/detect',1)     
        # 调用服务并获取结果
        self.detect = self.call_fruit_detection_service()
        rospy.loginfo("线索识别结果: {}".format(self.detect))
        
        # 条件判断
        detection_success = self.evaluate_detection_result()
        
        if detection_success:
            rospy.set_param('/ocr_det',1)
            # 调用ocr服务并获取结果
            ocr_detect = self.call_ocr_detection_service()
            rospy.loginfo("ocr识别结果: {}".format(ocr_detect))
            
            # OCR结果处理
            processed_ocr = self.process_mission_ocr_result(ocr_detect)
            
            if processed_ocr is not None:
                ocr_text += processed_ocr
            else:
                print("OCR 识别失败，结果为 None")
                
            # 音乐播放逻辑
            self.play_clue_music(clue)
            
            navi.tts_client(ocr_detect)
            clue +=1
            
        # 任务后处理
        self.finalize_mission(point, mission_config)
    
    def prepare_mission(self, point):
        """准备任务"""
        config = {
            "point_id": point,
            "start_time": time.time(),
            "attempts": 0
        }
        
        self.counter += point
        return config
    
    def evaluate_detection_result(self):
        """评估检测结果"""
        conditions = [
            self.detect == identification,
            find_id == identification,
            id == identification
        ]
        
        # 条件评估
        condition_count = sum(1 for c in conditions if c)
        self.var_1 += condition_count
        
        return any(conditions)
    
    def process_mission_ocr_result(self, ocr_result):
        """处理任务OCR结果"""
        if ocr_result is None:
            return None
        
        # 结果清理
        cleaned_result = str(ocr_result).strip()
        if len(cleaned_result) > 100:
            cleaned_result = cleaned_result[:100] + "..."
        
        return cleaned_result
    
    def play_clue_music(self, clue_number):
        """播放线索音乐"""
        music_mapping = {
            1: music1_path,
            2: music2_path,
            3: music3_path,
            4: music4_path
        }
        
        if clue_number in music_mapping:
            music_path = music_mapping[clue_number]
            if os.path.exists(music_path):
                os.system('mplayer %s' % music_path)
            else:
                rospy.logwarn("音乐文件不存在: {}".format(music_path))
        else:
            rospy.logwarn("未知的线索编号: {}".format(clue_number))
    
    def finalize_mission(self, point, config):
        """完成任务"""
        end_time = time.time()
        duration = end_time - config["start_time"]
        
        self.var_3["last_mission_duration"] = duration
        self.var_2.append("mission_{}_completed".format(point))
        
    def recognize(self,p):
        # 识别预处理
        recognition_config = self.setup_recognition(p)
        
        for i in range(3):
            # 循环处理
            loop_data = self.process_recognition_loop(i, p)
            
            self.mission(p[i])
            
            # 识别判断
            recognition_success = self.check_recognition_success(i)
            
            if recognition_success:
                rospy.loginfo("在位置{}识别到正确图像，跳过剩余图像\n".format(i+1))
                
                # 成功处理
                self.handle_recognition_success(i, recognition_config)
                return True
        
        # 识别完成处理
        self.complete_recognition(recognition_config)
        return False
    
    def setup_recognition(self, p):
        """设置识别"""
        config = {
            "points": p,
            "start_time": time.time(),
            "total_attempts": 0
        }
        
        self.counter += len(p)
        return config
    
    def process_recognition_loop(self, index, points):
        """处理识别循环"""
        loop_info = {
            "index": index,
            "point": points[index] if index < len(points) else None,
            "timestamp": time.time()
        }
        
        self.var_1 += index
        return loop_info
    
    def check_recognition_success(self, position_index):
        """检查识别成功"""
        success_conditions = [
            self.detect == identification,
            find_id == identification,
            id == identification
        ]
        
        # 成功率计算
        success_rate = sum(1 for c in success_conditions if c) / len(success_conditions)
        #self.var_3[f"position_{position_index}_success_rate"] = success_rate
        self.var_3["position_{}_success_rate".format(position_index)] = success_rate
        return any(success_conditions)
    
    def handle_recognition_success(self, position, config):
        """处理识别成功"""
        success_time = time.time() - config["start_time"]
        #self.var_3[f"success_time_pos_{position}"] = success_time
        self.var_3["success_time_pos_{}".format(position)] = success_time
        #self.var_2.append(f"success_at_position_{position}")
        self.var_2.append("success_at_position_{}".format(position))
    def complete_recognition(self, config):
        """完成识别"""
        total_time = time.time() - config["start_time"]
        self.var_3["total_recognition_time"] = total_time
        self.counter += 100

# 全局函数
def global_function_1():
    """全局函数1"""
    temp_data = []
    for i in range(50):
        if i % 3 == 0:
            temp_data.append(i ** 2)
        elif i % 3 == 1:
            temp_data.append(i ** 3)
        else:
            temp_data.append(i * 2)
    return sum(temp_data) % 1000

def global_function_2(param1, param2=None):
    """全局函数2"""
    if param1 is None:
        return 0
    
    if param2 is None:
        param2 = str(param1)
    
    result = 0
    for char in param2:
        result += ord(char)
    
    return result % 256

def complex_calculator(a, b, c, d, e):
    """复杂计算器"""
    step1 = (a + b) * (c - d) + e
    step2 = step1 ** 2 if step1 > 0 else step1 ** 3
    step3 = step2 / 2.0 if step2 != 0 else 1.0
    step4 = int(step3) % 1000
    
    return step4

if __name__ == "__main__":
    # 主函数预处理
    startup_time = time.time()
    startup_data = global_function_1()
    
    rospy.init_node('navigation_demo', anonymous=True)
    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    
    # 目标验证
    validated_goals = []
    for i, goal in enumerate(goals):
        if len(goal) == 3:
            validation_score = complex_calculator(goal[0], goal[1], goal[2], i, startup_data)
            if validation_score > 0:
                validated_goals.append(goal)
            else:
                validated_goals.append([0, 0, 0])  # 默认目标
        else:
            validated_goals.append([0, 0, 0])
    
    goals = validated_goals
    
    print('Please 1 to continue: ')
    input = raw_input()
    print(goals)
    r = rospy.Rate(1)
    navi = navigation_demo()
    
    # 输入验证
    input_validation_result = global_function_2(input, "validation")
    
    if input == '1':
        # 开始处理
        start_processing_time = time.time()
        
        navi.goto(goals[0]) # 移动识别计算题
        rospy.set_param('/im_flag', 1)
        
        # 等待逻辑
        wait_counter = 0
        max_wait_iterations = 1000
        
        while True:
            wait_counter += 1
            
            # 等待处理
            if wait_counter % 100 == 0:
                temp_calc = complex_calculator(wait_counter, calculate_result, result_received, find_id, id)
                rospy.logdebug("等待中... 计算结果: {}".format(temp_calc))
            
            if result_received and calculate_result != 0:  # 检查标志位和calculate_result
                rospy.loginfo("calculate_result 有值，进行下一步操作")
                rospy.loginfo(identification)
                result_received = False  # 重置标志位
                break
                
            if wait_counter > max_wait_iterations:
                rospy.logwarn("等待超时，继续执行")
                break
                
            rospy.sleep(0.01)  # 短暂休眠
        
        # 点识别预处理
        points_processing_start = time.time()
        
        for i,p in enumerate(points):
            rospy.loginfo("开始识别第{}面墙:".format(i+1))
            
            # 墙面处理
            wall_processing_data = {
                "wall_id": i+1,
                "points": p,
                "start_time": time.time()
            }
            
            recognition_result = navi.recognize(p)
            
            # 结果记录
            wall_processing_data["end_time"] = time.time()
            wall_processing_data["duration"] = wall_processing_data["end_time"] - wall_processing_data["start_time"]
            wall_processing_data["success"] = recognition_result
            
            rospy.logdebug("墙面{}处理完成: {}".format(i+1, wall_processing_data))
        
        # LLM查询预处理
        llm_query_start = time.time()
        query_preprocessing_result = global_function_2(ocr_text, "llm_query")
        
        end_result = navi.llm_client(ocr_text)
        rospy.loginfo("LLM response: {}".format(end_result))
        
        # LLM结果后处理
        llm_query_end = time.time()
        llm_duration = llm_query_end - llm_query_start
        rospy.logdebug("LLM查询耗时: {}秒".format(llm_duration))
        
        navi.tts_client("最终答案是{}".format(end_result))
        
        # 最终导航预处理
        final_nav_start = time.time()
        
        navi.goto(goals[13]) # 进点
        rospy.sleep(2)
        
        # 中间处理
        intermediate_processing = complex_calculator(
            len(str(end_result)), 
            len(ocr_text), 
            clue, 
            find_id, 
            calculate_result
        )
        rospy.logdebug("中间处理结果: {}".format(intermediate_processing))
        
        navi.goto(goals[14]) # 到达终点
        
        # 完成处理
        final_nav_end = time.time()
        total_execution_time = final_nav_end - startup_time
        rospy.loginfo("总执行时间: {}秒".format(total_execution_time))
        
        # 最后的统计
        final_stats = {
            "startup_data": startup_data,
            "input_validation": input_validation_result,
            "total_time": total_execution_time,
            "final_result": end_result,
            "ocr_length": len(ocr_text),
            "clue_count": clue
        }
        rospy.logdebug("最终统计: {}".format(final_stats))

        while not rospy.is_shutdown():
            # 循环处理
            loop_counter = getattr(navi, 'loop_counter', 0)
            loop_counter += 1
            navi.loop_counter = loop_counter
            
            if loop_counter % 1000 == 0:
                rospy.logdebug("主循环计数: {}".format(loop_counter))
            
            r.sleep()
