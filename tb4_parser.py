import rclpy
import time
import json
from std_msgs.msg import String
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from daim_llm.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
import threading
import math
from concurrent.futures import ThreadPoolExecutor
from rclpy.time import Time
from rclpy.duration import Duration
from actionlib_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from actionlib_msgs.msg import *
from copy import deepcopy
import numpy as np
import sys


# 로그 파일 경로 설정
error_path = '/path/to/error.txt'
command_path = '/path/to/command.txt'
cycle_path = '/path/to/cycle.txt'

# 파일 내용을 클리어하는 함수
def clear_storage(path):
    open(path, 'w').close()


class Giga(Node):
    # 클래스 초기화
    def __init__(self):
        super().__init__('Giga')
        self.create_subscription(String, '/voice_cmd', self.voice_cmd_callback, 10)
        

        # 위치 정보에 대한 구독 생성 및 기타 초기화
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)
        self.node = rclpy.create_node('map_navigation')
        self.ac = ActionClient(self, NavigateToPose, 'goal_pose')
        self.current_pose = PoseWithCovarianceStamped()
        self.action_lock = threading.Lock()
        self._stop = threading.Event()

        self.current_position = None
        self.target_position = None

        self.x = 0.0
        self.y = 0.0
        self.turning = False
        self.c_angle = 180

        self.thread_executor = ThreadPoolExecutor(max_workers=1)

        navigator = TurtleBot4Navigator()
        if navigator.getDockedStatus() :
            navigator.undock()
        print("Undocked")
    
        initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.SOUTH)
        navigator.setInitialPose(initial_pose)
	
		# Wait for Nav2
        navigator.waitUntilNav2Active()

        
# 로봇 이동 관련 파트
    # 위치 정보 업데이트 콜백 함수
    def callback(self, msg):
        self.current_pose = msg

    # Numpy 배열로 변환하는 유틸리티 함수
    def convert_to_nparray(self, o):
        return np.array([o.x, o.y, o.z, o.w])
    
    # 현재 위치 정보를 반환하는 함수
    def cur_pos_xyth(self, pos):
        x = pos.pose.pose.position.x
        y = pos.pose.pose.position.y
        
        tmp = self.convert_to_nparray(pos.pose.pose.orientation)
        [_,_, th_rad] = euler_from_quaternion(tmp)
        
        print (x, y, th_rad)
        return x, y, th_rad
    
    # 이동 명령을 처리하는 함수
    def move(self):
        navigator = TurtleBot4Navigator()

        print("go")

        goal_pose = navigator.getPoseStamped([-1*self.x, -1*self.y], TurtleBot4Directions.SOUTH) # map상에서 좌표계 방향성으로 인한 -1 부호 처리

        print(goal_pose)
        navigator.startToPose(goal_pose)
        print("Move Done")
        self.c_angle = 180

    # 회전 명령을 처리하는 함수
    def rotate_inplace_relative(self):

        navigator = TurtleBot4Navigator()
        goal = PoseStamped()

        a = self.current_pose.pose.pose.position.x
        b = self.current_pose.pose.pose.position.y

        if self.wise:
            desired_angle = (self.c_angle - self.ang) % 360
        else:
            desired_angle = (self.c_angle + self.ang) % 360

        goal_pose = navigator.getPoseStamped([a, b], desired_angle)

        print(goal_pose)
        navigator.startToPose(goal_pose)
        print("Rotate Done")
        self.c_angle = desired_angle

    # 상대적 위치로 이동하는 함수   
    def move_to_relative(self):
        
        l = self.dis # 원하는 이동거리

        [_, _, th] = self.cur_pos_xyth(self.current_pose)

        goal = PoseStamped()

        goal.pose.position.x = self.current_pose.pose.pose.position.x + l*math.cos(th)
        goal.pose.position.y = self.current_pose.pose.pose.position.y + l*math.sin(th)
        navigator = TurtleBot4Navigator()
        
        a = goal.pose.position.x
        b = goal.pose.position.y

        goal_pose = navigator.getPoseStamped([a, b], TurtleBot4Directions.SOUTH)

        print(goal_pose)
        navigator.startToPose(goal_pose)
        print("Move Done")
        self.c_angle = 180
    
    # def stop_movement(self):
    #     self.cancelTask()
    #     self.info("Turtlebot stopped.")
        

# 명령 실행 관련 파트
        
    # 에러 감지 및 처리 함수
    def error_detect(self, cmd):               
        if type(cmd) is str:                           # cmd 문자열인 경우 딕셔너리로 변형
            cmd = json.loads(cmd)
        missing = []

        # missing information 확인
        if cmd['action'] == 'move':
            distance = float(cmd['params'].get('distance'))
            if distance == 0 or distance == -9999999:
                missing.append("distance")

        elif cmd['action'] == 'rotate':
            angle = float(cmd['params'].get('angle'))
            is_clockwise = cmd['params'].get('is_clockwise' )
            if angle == 0 or angle==-9999999:
                missing.append("angle")
            if (is_clockwise != True and is_clockwise != False):
                missing.append("direction")

        elif cmd['action'] == 'go_to_goal':
            x = cmd['params']['location'].get('x')
            y = cmd['params']['location'].get('y')
            if x == -9999999:
                missing.append("x")
            if y == -9999999:
                missing.append("y")

        f = open(error_path, 'w')
        if len(missing) != 0 :                                  # missing information 있는 경우
            output = ', '.join(missing)
            print('[Error] Missing Information: ' + output)     # missing information 출력
            serialized = json.dumps(cmd)                        # error.txt 에 저장
            serialized = serialized.rstrip('\n')
            f.write(serialized)
            print("Saving error.txt")
        f.close()

    # 명령 실행 함수
    def execute(self, cmd):
        if type(cmd) is str:
            cmd = json.loads(cmd)

        if cmd['action'] == 'stop':
            self._stop.set()
            print("Execute: Stop")

        elif cmd['action'] == 'move':
            distance = float(cmd['params'].get('distance'))

            print("Execute: Move")
            self._stop.clear()
            self.dis = distance
            self.current_action = "move"

            self.thread_executor.submit(self.move_to_relative())

        elif cmd['action'] == 'rotate':
            angle = float(cmd['params'].get('angle'))
            is_clockwise = cmd['params'].get('is_clockwise')

            print("Execute: Rotate")
            self._stop.clear()
            self.ang = angle
            self.wise = is_clockwise
            self.current_action = "rotate"
           
            self.thread_executor.submit(self.rotate_inplace_relative())

        elif cmd['action'] == 'go_to_goal':
            self._stop.clear()
            self.x = float(cmd['params']['location'].get('x'))
            self.y = float(cmd['params']['location'].get('y'))

            print("Execute: go_to_goal")
            print(self.x, self.y)
            self.thread_executor.submit(self.move())

    # command.txt 파일의 한 줄을 실행하는 함수
    def execute_one_line(self):                 # command.txt 파일의 한 줄 삭제 후 실행
        
        ff = open(command_path, 'r')
        command_txt = ff.readlines()
        current_command = command_txt[0]
        del command_txt[0]                                           # 첫째줄 삭제
        ff.close()
        
        f = open(command_path, 'w')
        f.write("".join(command_txt))                                # 첫째줄 제외한 나머지 command 재작성
        f.close()

        self.error_detect(current_command)
        f = open(error_path, 'r')
        error = f.readline()
        f.close()
        if error == "":
            self.execute(current_command)
    
    # 명령 콜백 함수 (json formant에서 필요한 파라미터를 활용하여 명령 실행)
    def voice_cmd_callback(self, msg):
        # try:
            cmd = json.loads(msg.data)
            tmp = cmd['text']
            time.sleep(0.5)
            if tmp != "pass" :                              # msg가 pass가 아닐때 (command.txt 내용 없음)
                print('JSON command received: \n',cmd,'\n') 
                cmd = json.loads(cmd['text'])               # json commnd로 변환

                if cmd['action'] == 'sequence':             # action이 sequence: command.txt에 저장
                    serialized = json.dumps(cmd)    
                    serialized = serialized[34:-2]
                    split_text = serialized.split("}, {")
                    output = "}\n{".join(split_text) + "\nend\n"
                    f = open(command_path, 'w')
                    f.write(output)
                    f.close()
                    print("Sequence saved.")
                else:                                       # sequence 아니면 gpt로부터 넘겨받은 msg 실행
                    self.error_detect(cmd)
                    f = open(error_path, 'r')
                    error = f.readline()
                    f.close()

                    if error == "": 
                        f = open(command_path, 'r')
                        command = f.readlines()
                        command = [line.rstrip('\n') for line in command] 
                        f.close()
                        

                        if command :
                            f = open(command_path, 'w')
                            save = "\n".join(command) + '\n' + json.dumps(cmd) + '\n'
                            f.write(save)
                            f.close()
                        else :
                            self.execute(cmd)
                    else : 
                        f = open(error_path, 'w')
                        serialized = json.dumps(cmd) 
                        serialized = serialized.rstrip('\n')                 
                        f.write(serialized)
                        f.close()

            else:                                           # msg가 pass 일때 (command.txt 내용 있음)
                f = open(command_path, 'r')
                commands = f.readlines()
                commands = [line.rstrip('\n') for line in commands] 
                f.close()        

                print(commands)
                if commands[0] == "end":                                      # Sequentail Command Error Feedback이 종료된 경우
                    del commands[0]                                           # 첫째줄 삭제   
                    for i in range (len(commands)):
                        
                        with self.action_lock:
                            if commands[i] != "{\"action\": \"stop\"}":
                                self.execute(commands[i])
                    

                    clear_storage(command_path)
                    print("exe done")

                else: 
                    correction = commands[0]
                    del commands[0]                                           # 첫째줄 삭제                
                    self.error_detect(correction)

                    f = open(error_path, 'r')
                    error = f.readline()
                    
                    f.close()
                    if error == "":
                        f = open(command_path, 'w')
                        f.write("\n".join(commands) + '\n' + correction +'\n')
                        f.close()
                    else:
                        f = open(command_path, 'w')
                        f.write("\n".join(commands))                             # 첫째줄 제외한 나머지 command 재작성
                        f.close()

            clear_storage(cycle_path)


        
# 메인 함수 정의
def main(args=None):
    rclpy.init(args=args)
    node = Giga()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()