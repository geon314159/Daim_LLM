import json
import rclpy
from rclpy.node import Node
import requests

# 로그 파일 경로 설정
error_path = '/path/to/error.txt'
command_path = '/path/to/command.txt'
cycle_path = '/path/to/cycle.txt'

# 파일 내용을 클리어하는 함수
def clear_storage(path):
    open(path, 'w').close()

# ROS2 노드를 상속받아 ChatGPT 클라이언트 기능을 구현하는 클래스
class ChatGPTClient(Node):
    # 클래스 생성자, 노드 초기화 및 서버 URL 설정
    def __init__(self):
        super().__init__('rosgpt_client')
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value
        self.get_logger().info('DAIM-LLM client started')

        # 로그 파일 초기화
        clear_storage(error_path)
        clear_storage(command_path)
        clear_storage(cycle_path)

        # 텍스트 명령 전송 기능 실행
        self.send_text_command()

    # 서버에 텍스트 명령을 전송하고 응답을 처리하는 함수
    def send_text_command(self):
        while rclpy.ok():
            f = open(cycle_path, 'r')
            ff = f.readline()
            f.close()
            if ff == "":
                a = open(cycle_path, 'w')
                a.write("Runnig...")
                a.close()

                f = open(error_path, 'r')
                insuff_info = f.readline()
                f.close()

                text_command = "" 
                if not insuff_info == "":                           # error.txt 차있을때
                    dict = json.loads(insuff_info)                  # dict에 error 저장

                    missing=[]
                    error_msg = "" 
                    if dict.get('action') == 'move':                # missing information 확인
                        for key, value in dict["params"].items():
                            if value == -9999999 or value == 0 :
                                missing.append(key)
                    if dict.get('action') == 'rotate':
                        for key, value in dict["params"].items():
                            if value == -9999999 or value == 0 :
                                missing.append(key)
                    if dict.get('action') == 'go_to_goal':
                        for key, value in dict["params"]['location'].items():
                            if value == -9999999:
                                missing.append(key)
                    for i in range(len(missing)):
                        error_msg = error_msg + missing[i]
                        if dict.get('action') == 'go_to_goal':
                            error_msg = error_msg + " coordinate"
                        if i != len(missing)-1 : 
                            error_msg = error_msg + " and "
                        else: 
                            error_msg = error_msg + "."

                    print("Your Command: " + insuff_info)              # 수정중인 command 설명
                    print("We need information about " + error_msg)    # missing information 설명

                    text_command = input("Enter a text command. Or type \"Y\" to stop the robot.: ")            # info 추가할 경우 저장
                    if text_command=="Y" or text_command=="y" or text_command=="yes" or text_command=="Yes":    # restart 명령 받은 경우
                        clear_storage(error_path)                                                               # error 파일 삭제
                        print("The robot is stopped. Restarting...")
                        print("\nReady!")                                                   # restart 된 경우 
                        text_command = input("Enter a command: ")

                else:                                                       # error.txt 비어있을때                     
                    f = open(command_path, 'r')                             # command.txt 파일 불러오기
                    command = f.readlines()                                 # command.txt 파일에서 첫번째줄 읽기
                    f.close()
                    if not command :                                        # command.txt, error.txt 둘다 비어있을때
                        print("\nReady!")
                        text_command = input("Enter a text command: ")      # new command 받기
                    else:                                                   # command.txt 차있고 error.txt 비어있을때
                        text_command = "pass"                               # command.txt 파일 실행할 수 있도록 pass 전달 (proxy에서 "pass" command 처리)

                data = {'text_command': text_command}
                response = requests.post(self.server_url, data=data)
                if response.status_code == 200:
                    try:
                        response_str = response.content.decode('utf-8')
                        response_dict = json.loads(response_str)
                    except Exception as e:
                        print('[Exception] An unexpected error occurred:', str(e))
                else:
                    self.get_logger().error('Error: {}'.format(response.status_code))

# 메인 함수 정의
def main(args=None):
    rclpy.init(args=args)
    daim_llm_client = ChatGPTClient()
    rclpy.spin(daim_llm_client)
    daim_llm_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()