"""
로봇 관절값 로깅 프로그램
로봇이 이동하는 동안 실시간으로 관절값과 좌표값을 기록합니다
"""

from dobot_api import DobotApiDashboard, DobotApiFeedBack
from time import sleep
import threading
import time
from datetime import datetime
import csv
import os

class RobotPositionLogger:
    def __init__(self, ip="192.168.5.1"):
        self.ip = ip
        self.dashboard = None
        self.feedback = None
        self.current_mode = 0
        self.current_command_id = 0
        
        # 로깅 관련
        self.logging_active = False
        self.log_data = []
        self.log_thread = None
        
    def connect(self):
        """로봇 연결"""
        print("=" * 70)
        print("🤖 로봇 연결 중...")
        print("=" * 70)
        
        self.dashboard = DobotApiDashboard(self.ip, 29999)
        self.feedback = DobotApiFeedBack(self.ip, 30004)
        
        print("✅ 연결 완료")
        
        # 피드백 스레드 시작
        feed_thread = threading.Thread(target=self.get_feedback)
        feed_thread.daemon = True
        feed_thread.start()
        sleep(1)
        
    def get_feedback(self):
        """실시간 피드백 수신"""
        while True:
            try:
                data = self.feedback.feedBackData()
                if data:
                    self.current_mode = data.get('RobotMode', [0])[0]
                    self.current_command_id = data.get('CurrentCommandId', [0])[0]
                sleep(0.01)
            except:
                break
    
    def enable_robot(self):
        """로봇 활성화"""
        print("\n[로봇 활성화]")
        
        # 1. 에러 클리어
        print("  1. 에러 클리어...")
        self.dashboard.ClearError()
        sleep(0.5)
        
        # 2. 제어권 요청
        print("  2. 제어권 요청...")
        result = self.dashboard.RequestControl()
        print(f"     결과: {result}")
        sleep(0.5)
        
        # 3. Enable
        print("  3. Enable...")
        result = self.dashboard.EnableRobot()
        print(f"     결과: {result}")
        
        if "0" in result:
            print("  ✅ 로봇 활성화 성공!")
            sleep(1)
            return True
        else:
            print(f"  ❌ 활성화 실패: {result}")
            return False
    
    def start_logging(self, position_name):
        """로깅 시작"""
        self.logging_active = True
        self.log_data = []
        
        print(f"\n📝 로깅 시작: {position_name}")
        
        # 로깅 스레드 시작
        self.log_thread = threading.Thread(
            target=self._logging_worker, 
            args=(position_name,)
        )
        self.log_thread.daemon = True
        self.log_thread.start()
    
    def stop_logging(self):
        """로깅 종료"""
        self.logging_active = False
        if self.log_thread:
            self.log_thread.join(timeout=1.0)
        print(f"📝 로깅 종료 (총 {len(self.log_data)}개 데이터 수집)")
    
    def _logging_worker(self, position_name):
        """로깅 작업 스레드"""
        start_time = time.time()
        
        while self.logging_active:
            try:
                # 피드백 데이터 수집
                data = self.feedback.feedBackData()
                
                if data is not None and len(data) > 0:
                    # TestValue 확인 (데이터 유효성)
                    if hex((data['TestValue'][0])) == '0x123456789abcdef':
                        current_time = time.time() - start_time
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        
                        # 관절 각도 (QActual) - 올바른 2차원 배열 인덱싱
                        joints = data['QActual'][0]  # (1, 6) -> (6,)
                        j1 = joints[0]
                        j2 = joints[1]
                        j3 = joints[2]
                        j4 = joints[3]
                        j5 = joints[4]
                        j6 = joints[5]
                        
                        # 직교 좌표 (ToolVectorActual) - 올바른 2차원 배열 인덱싱
                        pose = data['ToolVectorActual'][0]  # (1, 6) -> (6,)
                        x = pose[0]
                        y = pose[1]
                        z = pose[2]
                        rx = pose[3]
                        ry = pose[4]
                        rz = pose[5]
                        
                        # 로봇 모드
                        robot_mode = data['RobotMode'][0]
                        
                        # 로그 데이터 저장
                        log_entry = {
                            'position_name': position_name,
                            'timestamp': timestamp,
                            'elapsed_time': f"{current_time:.3f}",
                            'j1': f"{j1:.4f}",
                            'j2': f"{j2:.4f}",
                            'j3': f"{j3:.4f}",
                            'j4': f"{j4:.4f}",
                            'j5': f"{j5:.4f}",
                            'j6': f"{j6:.4f}",
                            'x': f"{x:.4f}",
                            'y': f"{y:.4f}",
                            'z': f"{z:.4f}",
                            'rx': f"{rx:.4f}",
                            'ry': f"{ry:.4f}",
                            'rz': f"{rz:.4f}",
                            'robot_mode': robot_mode
                        }
                        
                        self.log_data.append(log_entry)
                
                sleep(0.05)  # 20Hz 샘플링 (초당 20번)
                
            except Exception as e:
                print(f"⚠️ 로깅 오류: {e}")
                break
    
    def save_log_to_csv(self, filename=None):
        """로그를 CSV 파일로 저장"""
        if not self.log_data:
            print("⚠️ 저장할 로그 데이터가 없습니다.")
            return None
        
        # 파일명 생성
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_log_{timestamp}.csv"
        
        # logs 폴더 생성
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        filepath = os.path.join(log_dir, filename)
        
        # CSV 저장
        try:
            with open(filepath, 'w', newline='', encoding='utf-8-sig') as csvfile:
                fieldnames = [
                    'position_name', 'timestamp', 'elapsed_time',
                    'j1', 'j2', 'j3', 'j4', 'j5', 'j6',
                    'x', 'y', 'z', 'rx', 'ry', 'rz',
                    'robot_mode'
                ]
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.log_data)
            
            print(f"\n💾 로그 저장 완료: {filepath}")
            print(f"   총 {len(self.log_data)}개 데이터 포인트")
            return filepath
            
        except Exception as e:
            print(f"❌ 로그 저장 실패: {e}")
            return None
    
    def append_log_to_csv(self, filepath):
        """기존 CSV 파일에 로그 추가"""
        if not self.log_data:
            return
        
        try:
            with open(filepath, 'a', newline='', encoding='utf-8-sig') as csvfile:
                fieldnames = [
                    'position_name', 'timestamp', 'elapsed_time',
                    'j1', 'j2', 'j3', 'j4', 'j5', 'j6',
                    'x', 'y', 'z', 'rx', 'ry', 'rz',
                    'robot_mode'
                ]
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerows(self.log_data)
            
            print(f"💾 로그 추가 완료: +{len(self.log_data)}개 데이터")
            
        except Exception as e:
            print(f"❌ 로그 추가 실패: {e}")
    
    def move_to_joint_position_with_log(self, name, j1, j2, j3, j4, j5, j6, speed=50, accel=50):
        """
        관절 좌표로 이동하며 로그 기록
        """
        print(f"\n▶ [{name}] 관절 좌표로 이동 중...")
        print(f"   J1={j1:.3f}°, J2={j2:.3f}°, J3={j3:.3f}°")
        print(f"   J4={j4:.3f}°, J5={j5:.3f}°, J6={j6:.3f}°")
        
        # 로깅 시작
        self.start_logging(name)
        
        # MovJ - coordinateMode=1 (Joint 모드)
        result = self.dashboard.MovJ(j1, j2, j3, j4, j5, j6, 1, v=speed, a=accel)
        print(f"   명령 전송: {result}")
        
        if "Not Tcp" in result:
            print("   ❌ TCP 모드가 아닙니다!")
            self.stop_logging()
            return False
        
        # 명령 ID 추출
        import re
        match = re.search(r'(\d+)', result)
        if match:
            cmd_id = int(match.group(1))
            
            # 이동 완료 대기
            print("   이동 중...", end="", flush=True)
            while True:
                if self.current_mode == 5 and self.current_command_id == cmd_id:
                    print(" 완료!")
                    break
                print(".", end="", flush=True)
                sleep(0.2)
            
            sleep(0.5)
            
            # 로깅 종료
            self.stop_logging()
            return True
        
        self.stop_logging()
        return False
    
    def move_to_cartesian_position_with_log(self, name, x, y, z, rx, ry, rz, speed=50, accel=50):
        """
        직교 좌표로 이동하며 로그 기록
        """
        print(f"\n▶ [{name}] 직교 좌표로 이동 중...")
        print(f"   X={x:.2f}mm, Y={y:.2f}mm, Z={z:.2f}mm")
        print(f"   Rx={rx:.2f}°, Ry={ry:.2f}°, Rz={rz:.2f}°")
        
        # 로깅 시작
        self.start_logging(name)
        
        # MovJ - coordinateMode=0 (Pose 모드)
        result = self.dashboard.MovJ(x, y, z, rx, ry, rz, 0, v=speed, a=accel)
        print(f"   명령 전송: {result}")
        
        if "Not Tcp" in result:
            print("   ❌ TCP 모드가 아닙니다!")
            self.stop_logging()
            return False
        
        # 명령 ID 추출
        import re
        match = re.search(r'(\d+)', result)
        if match:
            cmd_id = int(match.group(1))
            
            # 이동 완료 대기
            print("   이동 중...", end="", flush=True)
            while True:
                if self.current_mode == 5 and self.current_command_id == cmd_id:
                    print(" 완료!")
                    break
                print(".", end="", flush=True)
                sleep(0.2)
            
            sleep(0.5)
            
            # 로깅 종료
            self.stop_logging()
            return True
        
        self.stop_logging()
        return False
    
    def play_and_log_joint_positions(self):
        """관절 좌표로 이동하며 전체 로그 기록"""
        print("\n" + "=" * 70)
        print("🎬 캡처된 위치 재생 및 로깅 시작 (관절 좌표)")
        print("=" * 70)
        
        # 로그 파일 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"robot_log_joint_{timestamp}.csv"
        log_filepath = None
        
        # 이동할 포지션들
        positions = [
            {
                "name": "Position 1",
                "j1": -70.197, "j2": -27.691, "j3": -4.1231,
                "j4": -17.403, "j5": 93.926, "j6": 29.172
            },
            {
                "name": "Position 2",
                "j1": -70.041, "j2": -21.591, "j3": -17.52,
                "j4": -11.30, "j5": 93.830, "j6": 29.289
            },
            {
                "name": "Position 3",
                "j1": -70.061, "j2": -18.345, "j3": -25.66,
                "j4": -5.715, "j5": 93.903, "j6": 29.616
            },
            {
                "name": "Position 4",
                "j1": -71.4114, "j2": -74.3324, "j3": -39.2727,
                "j4": 33.3140, "j5": 91.6116, "j6": 8.2885
            }
        ]
        
        for i, pos in enumerate(positions):
            success = self.move_to_joint_position_with_log(
                pos["name"],
                pos["j1"], pos["j2"], pos["j3"],
                pos["j4"], pos["j5"], pos["j6"],
                speed=30,
                accel=30
            )
            
            if not success:
                print(f"❌ {pos['name']} 이동 실패")
                break
            
            # 첫 번째 로그는 새 파일로, 이후는 추가
            if i == 0:
                log_filepath = self.save_log_to_csv(log_filename)
            else:
                if log_filepath:
                    self.append_log_to_csv(log_filepath)
            
            sleep(1)  # 각 포인트에서 1초 대기
        
        print("\n" + "=" * 70)
        print("✅ 모든 포인트 이동 및 로깅 완료!")
        print("=" * 70)
        
        if log_filepath:
            print(f"\n📊 로그 파일 위치: {log_filepath}")
    
    def play_and_log_cartesian_positions(self):
        """직교 좌표로 이동하며 전체 로그 기록"""
        print("\n" + "=" * 70)
        print("🎬 캡처된 위치 재생 및 로깅 시작 (직교 좌표)")
        print("=" * 70)
        
        # 로그 파일 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"robot_log_cartesian_{timestamp}.csv"
        log_filepath = None
        
        # 이동할 포지션들
        positions = [
            {
                "name": "Position 1",
                "x": 27.568, "y": -302.4, "z": 481.98,
                "rx": -144.11, "ry": 21.325, "rz": 179.05
            },
            {
                "name": "Position 2",
                "x": 27.774, "y": -301.11, "z": 476.00,
                "rx": -145.31, "ry": 20.895, "rz": 178.62
            },
            {
                "name": "Position 3",
                "x": 27.777, "y": -301.11, "z": 472.06,
                "rx": -144.8, "ry": 21.384, "rz": 178.60
            },
            {
                "name": "Position 4",
                "x": 65.0080, "y": -442.2058, "z": 100.7271,
                "rx": -170.6129, "ry": 2.9659, "rz": -169.3195
            }
        ]
        
        for i, pos in enumerate(positions):
            success = self.move_to_cartesian_position_with_log(
                pos["name"],
                pos["x"], pos["y"], pos["z"],
                pos["rx"], pos["ry"], pos["rz"],
                speed=30,
                accel=30
            )
            
            if not success:
                print(f"❌ {pos['name']} 이동 실패")
                break
            
            # 첫 번째 로그는 새 파일로, 이후는 추가
            if i == 0:
                log_filepath = self.save_log_to_csv(log_filename)
            else:
                if log_filepath:
                    self.append_log_to_csv(log_filepath)
            
            sleep(1)  # 각 포인트에서 1초 대기
        
        print("\n" + "=" * 70)
        print("✅ 모든 포인트 이동 및 로깅 완료!")
        print("=" * 70)
        
        if log_filepath:
            print(f"\n📊 로그 파일 위치: {log_filepath}")


def main():
    print("\n📝 로봇 관절값 로깅 프로그램")
    print()
    print("로봇이 4개의 위치로 이동하며 실시간 관절값을 기록합니다.")
    print("로그는 CSV 파일로 저장되어 나중에 분석할 수 있습니다.")
    print()
    
    # 모드 선택
    print("이동 모드를 선택하세요:")
    print("  1. 관절 좌표 (Joint) - J1~J6 값 사용")
    print("  2. 직교 좌표 (Cartesian) - X,Y,Z,Rx,Ry,Rz 값 사용")
    print()
    
    while True:
        choice = input("선택 (1 또는 2): ").strip()
        if choice in ['1', '2']:
            break
        print("1 또는 2를 입력하세요.")
    
    print()
    
    try:
        logger = RobotPositionLogger()
        
        # 연결
        logger.connect()
        
        # 활성화
        if not logger.enable_robot():
            print("\n로봇 활성화 실패. 프로그램을 종료합니다.")
            return
        
        # 선택한 모드로 재생 및 로깅
        if choice == '1':
            logger.play_and_log_joint_positions()
        else:
            logger.play_and_log_cartesian_positions()
        
        print("\n프로그램을 종료합니다.")
        
    except KeyboardInterrupt:
        print("\n\n사용자가 중단했습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
