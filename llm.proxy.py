import os
import json
import openai
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, send_from_directory, jsonify
from flask_restful import Resource, Api
from flask_cors import CORS
import pyttsx3
from rclpy.executors import SingleThreadedExecutor
from ament_index_python import get_package_share_directory

# Flask 애플리케이션 인스턴스 생성 및 CORS 설정
app = Flask(__name__)
CORS(app)
api = Api(app)

# OpenAI API 키 설정
openai.api_key = 'your_openai_api_key'

# 스레딩을 위한 락 객체 생성
spin_lock = threading.Lock()
tts_lock = threading.Lock()

# 로그 파일 경로 설정
error_path = '/path/to/error.txt'
command_path = '/path/to/command.txt'
cycle_path = '/path/to/cycle.txt'

# 파일 내용을 클리어하는 함수
def clear_storage(file_path):
    open(file_path, 'w').close()

# ROS2 노드 클래스 정의
class ChatGPTNode(Node):
    def __init__(self):
        super().__init__('chatgpt_ros2_node')
        self.publisher = self.create_publisher(String, 'voice_cmd', 10)

    # 메시지를 ROS2 토픽으로 발행하는 함수
    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

# ChatGPT 응답을 처리하고 ROS2 노드로 발행하는 함수
def process_and_publish_chatgpt_response(chatgpt_ros2_node, text_command, chatgpt_response, use_executors=True):
    chatgpt_ros2_node.publish_message(chatgpt_response)
    if use_executors:
        executor = SingleThreadedExecutor()
        executor.add_node(chatgpt_ros2_node)
        executor.spin_once()
        executor.remove_node(chatgpt_ros2_node)
    else:
        with spin_lock:
            rclpy.spin_once(chatgpt_ros2_node)

# Flask RESTful 리소스 클래스 정의
class ChatGPTProxy(Resource):
    def __init__(self, chatgpt_ros2_node):
        self.chatgpt_ros2_node = chatgpt_ros2_node

    # ChatGPT에 질문을 하고 응답을 처리하는 함수
    def askGPT(self, text_command):
        # 중략: OpenAI와 통신하여 응답을 처리하는 코드

        prompt = '''중략'''
        
        error_promt = '''중략'''

        f = open(error_path, 'r')
        previous_info = f.readline()
        f.close()
        
        if previous_info == "":           # error.txt 비어있는 경우 처리
            prompt = prompt +'\nprompt: ' + text_command
            messages = [
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt}
            ]
        else:                             # error.txt 차있는 경우 처리
            prompt =  error_promt + '\nprompt: ' + text_command
            messages = [
                {"role": "system", "content": "You are a helpful assistant. The previous information is: " + previous_info + "Return previous info with changing the value acording to the promt below."},
                {"role": "user", "content": prompt}
            ]

        clear_storage(error_path)

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=messages,
            )
        except openai.error.InvalidRequestError as e:
            print(f"Error: {e}")
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
            return None

        chatgpt_response = response.choices[0].message['content'].strip()
        start_index = chatgpt_response.find('{')
        end_index = chatgpt_response.rfind('}') + 1
        json_response_dict = chatgpt_response[start_index:end_index]
        return json.dumps({'text': chatgpt_response})

    # HTTP POST 요청을 처리하는 함수
    def post(self):
        text_command = request.form['text_command']

        if text_command == "pass": # Sequential Command 를 처리하는 경우
            print("Got text command: pass")
            chatgpt_not_response = json.dumps({'text': 'pass'})
            threading.Thread(target=process_and_publish_chatgpt_response, args=(self.chatgpt_ros2_node, "pass", chatgpt_not_response, True)).start()
            return json.loads(chatgpt_not_response)

        else: # Sequential Command 를 처리하지 않는 경우
            print ('[DAIMLLM] Command received. ', text_command, '. Asking ChatGPT ...')
            chatgpt_response = self.askGPT(text_command)
            print ('[DAIMLLM] Response received from ChatGPT. \n', str(json.loads(chatgpt_response)))

            if chatgpt_response is None:
                return {'error': 'An error occurred while processing the request'}

            threading.Thread(target=process_and_publish_chatgpt_response, args=(self.chatgpt_ros2_node, text_command, chatgpt_response, True)).start()
            
        return json.loads(chatgpt_response)

# Flask 라우팅 설정
@app.route('/')
def index():
    return send_from_directory(os.path.join(get_package_share_directory('rosgpt'), 'webapp'), 'index.html')

# 메인 함수 정의
def main():
    rclpy.init(args=None)
    chatgpt_ros2_node = ChatGPTNode()
    api.add_resource(ChatGPTProxy, '/rosgpt', resource_class_args=(chatgpt_ros2_node,))
    app.run(debug=True, host='0.0.0.0', port=5000)
    chatgpt_ros2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()