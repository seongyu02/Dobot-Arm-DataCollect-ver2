"""
수동 티칭 로깅 프로그램
로봇을 수동으로 움직이는 동안 실시간으로 관절값을 기록합니다.

사용 방법:
1. 프로그램 실행
2. 로봇을 수동으로 원하는 위치로 이동
3. 'r' 키를 눌러 로깅 시작
4. 로봇을 계속 움직임
5. 's' 키를 눌러 로깅 중지 및 저장
6. 'q' 키를 눌러 프로그램 종료
"""

from dobot_api import DobotApiDashboard, DobotApiFeedBack
from time import sleep
import threading
import time
from datetime import datetime
import csv
import os
import sys

# Windows에서 키 입력을 위한 모듈
try:
    import msvcrt
except ImportError:
    print("이 프로그램은 Windows에서만 작동합니다.")
    sys.exit(1)


class ManualTeachingLogger:
    def __init__(self, ip="192.168.5.1"):
        self.ip = ip
        self.dashboard = None
        self.feedback = None
        
        # 로깅 관련
        self.logging_active = False
        self.log_data = []
        self.log_thread = None
        self.teaching_session = 0
        
        # 현재 로봇 상태
        self.current_joints = [0] * 6
        self.current_pose = [0] * 6
        self.current_mode = 0
        
    def connect(self):
        """로봇 연결"""
        print("=" * 70)
        print("🤖 로봇 연결 중...")
        print("=" * 70)
        
        try:
            self.dashboard = DobotApiDashboard(self.ip, 29999)
            self.feedback = DobotApiFeedBack(self.ip, 30004)
            
            print("✅ 연결 완료")
            
            # 피드백 스레드 시작
            feed_thread = threading.Thread(target=self.get_feedback)
            feed_thread.daemon = True
            feed_thread.start()
            sleep(1)
            
            return True
            
        except Exception as e:
            print(f"❌ 연결 실패: {e}")
            return False
    
    def get_feedback(self):
        """실시간 피드백 수신 (백그라운드용)"""
        while True:
            try:
                data = self.feedback.feedBackData()
                if data is not None and len(data) > 0:
                    # TestValue로 데이터 유효성 확인
                    if hex(data['TestValue'][0]) == '0x123456789abcdef':
                        # 관절 각도 (QActual)
                        self.current_joints = data['QActual'][0].tolist()
                        
                        # 직교 좌표 (ToolVectorActual)
                        self.current_pose = data['ToolVectorActual'][0].tolist()
                        
                        # 로봇 모드
                        self.current_mode = data['RobotMode'][0]
                
                sleep(0.01)
            except:
                break
    
    def start_logging(self, session_name=None):
        """로깅 시작"""
        if self.logging_active:
            print("\n⚠️  이미 로깅이 진행 중입니다.")
            return
        
        self.teaching_session += 1
        
        if session_name is None:
            session_name = f"Session_{self.teaching_session}"
        
        self.logging_active = True
        self.log_data = []
        
        print("\n" + "=" * 70)
        print(f"📝 로깅 시작: {session_name}")
        print("=" * 70)
        print("💡 로봇을 원하는 대로 움직이세요.")
        print("   's' 키를 누르면 로깅이 중지되고 저장됩니다.")
        print("=" * 70)
        
        # 로깅 스레드 시작
        self.log_thread = threading.Thread(
            target=self._logging_worker, 
            args=(session_name,)
        )
        self.log_thread.daemon = True
        self.log_thread.start()
    
    def stop_logging(self):
        """로깅 종료"""
        if not self.logging_active:
            print("\n⚠️  로깅이 진행 중이 아닙니다.")
            return
        
        self.logging_active = False
        if self.log_thread:
            self.log_thread.join(timeout=1.0)
        
        print("\n" + "=" * 70)
        print(f"📝 로깅 종료 (총 {len(self.log_data)}개 데이터 수집)")
        print("=" * 70)
        
        # 자동 저장
        if self.log_data:
            self.save_log_to_csv()
    
    def _logging_worker(self, session_name):
        """로깅 작업 스레드"""
        start_time = time.time()
        
        while self.logging_active:
            try:
                # 피드백 데이터 직접 수집
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
                        
                        # ToolDO 상태 조회 (석션/그리퍼)
                        # ToolDO는 DigitalOutputs에 포함되지 않으므로 별도 조회
                        try:
                            tooldo1_result = self.dashboard.GetToolDO(1)
                            # 디버그: 실제 응답 출력 (첫 10개만)
                            if len(self.log_data) < 10:
                                print(f"\n[DEBUG] GetToolDO(1) 응답: {tooldo1_result}")
                            
                            # 여러 가능한 응답 형식 체크
                            if ',{1},' in tooldo1_result or ',{1}' in tooldo1_result or '{1}' in tooldo1_result:
                                tooldo1 = 1
                            elif ',{0},' in tooldo1_result or ',{0}' in tooldo1_result or '{0}' in tooldo1_result:
                                tooldo1 = 0
                            else:
                                tooldo1 = 0
                        except Exception as e:
                            if len(self.log_data) < 10:
                                print(f"\n[DEBUG] GetToolDO(1) 오류: {e}")
                            tooldo1 = 0
                        
                        try:
                            tooldo2_result = self.dashboard.GetToolDO(2)
                            # 디버그: 실제 응답 출력 (첫 10개만)
                            if len(self.log_data) < 10:
                                print(f"[DEBUG] GetToolDO(2) 응답: {tooldo2_result}")
                            
                            # 여러 가능한 응답 형식 체크
                            if ',{1},' in tooldo2_result or ',{1}' in tooldo2_result or '{1}' in tooldo2_result:
                                tooldo2 = 1
                            elif ',{0},' in tooldo2_result or ',{0}' in tooldo2_result or '{0}' in tooldo2_result:
                                tooldo2 = 0
                            else:
                                tooldo2 = 0
                        except Exception as e:
                            if len(self.log_data) < 10:
                                print(f"[DEBUG] GetToolDO(2) 오류: {e}")
                            tooldo2 = 0
                        
                        # 일반 DO도 함께 기록 (참고용)
                        digital_outputs = int(data['DigitalOutputs'][0])
                        do1 = (digital_outputs >> 0) & 1
                        do2 = (digital_outputs >> 1) & 1
                        
                        # 로그 데이터 저장
                        log_entry = {
                            'session_name': session_name,
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
                            'robot_mode': robot_mode,
                            'tooldo1': tooldo1,  # ToolDO1 (주 석션/그리퍼)
                            'tooldo2': tooldo2,  # ToolDO2
                            'do1': do1,          # 일반 DO1 (참고용)
                            'do2': do2           # 일반 DO2 (참고용)
                        }
                        
                        self.log_data.append(log_entry)
                
                sleep(0.05)  # 20Hz 샘플링
                
            except Exception as e:
                print(f"⚠️ 로깅 오류: {e}")
                import traceback
                traceback.print_exc()
                break
    
    def save_log_to_csv(self, filename=None):
        """로그를 CSV 파일로 저장"""
        if not self.log_data:
            print("⚠️ 저장할 로그 데이터가 없습니다.")
            return None
        
        # 파일명 생성
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"manual_teaching_{timestamp}.csv"
        
        # logs 폴더 생성
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        filepath = os.path.join(log_dir, filename)
        
        # CSV 저장
        try:
            with open(filepath, 'w', newline='', encoding='utf-8-sig') as csvfile:
                fieldnames = [
                    'session_name', 'timestamp', 'elapsed_time',
                    'j1', 'j2', 'j3', 'j4', 'j5', 'j6',
                    'x', 'y', 'z', 'rx', 'ry', 'rz',
                    'robot_mode',
                    'tooldo1', 'tooldo2',  # ToolDO (석션/그리퍼)
                    'do1', 'do2'           # 일반 DO (참고용)
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
    
    def display_current_position(self):
        """현재 위치 표시"""
        try:
            # 최신 피드백 데이터 직접 읽기
            data = self.feedback.feedBackData()
            
            if data is not None and len(data) > 0:
                # TestValue 확인
                if hex((data['TestValue'][0])) == '0x123456789abcdef':
                    joints = data['QActual'][0]  # (1, 6) -> (6,)
                    pose = data['ToolVectorActual'][0]  # (1, 6) -> (6,)
                    mode = data['RobotMode'][0]
                    
                    # ToolDO 상태 조회
                    try:
                        tooldo1_result = self.dashboard.GetToolDO(1)
                        print(f"[DEBUG] GetToolDO(1) 응답: {tooldo1_result}")
                        
                        # 여러 가능한 응답 형식 체크
                        if ',{1},' in tooldo1_result or ',{1}' in tooldo1_result or '{1}' in tooldo1_result:
                            tooldo1 = 1
                        elif ',{0},' in tooldo1_result or ',{0}' in tooldo1_result or '{0}' in tooldo1_result:
                            tooldo1 = 0
                        else:
                            tooldo1 = -1
                    except Exception as e:
                        print(f"[DEBUG] GetToolDO(1) 오류: {e}")
                        tooldo1 = -1
                    
                    try:
                        tooldo2_result = self.dashboard.GetToolDO(2)
                        print(f"[DEBUG] GetToolDO(2) 응답: {tooldo2_result}")
                        
                        # 여러 가능한 응답 형식 체크
                        if ',{1},' in tooldo2_result or ',{1}' in tooldo2_result or '{1}' in tooldo2_result:
                            tooldo2 = 1
                        elif ',{0},' in tooldo2_result or ',{0}' in tooldo2_result or '{0}' in tooldo2_result:
                            tooldo2 = 0
                        else:
                            tooldo2 = -1
                    except Exception as e:
                        print(f"[DEBUG] GetToolDO(2) 오류: {e}")
                        tooldo2 = -1
                    
                    print("\n" + "-" * 70)
                    print("📍 현재 로봇 위치:")
                    print(f"   관절각: J1={joints[0]:.3f}°, J2={joints[1]:.3f}°, J3={joints[2]:.3f}°")
                    print(f"           J4={joints[3]:.3f}°, J5={joints[4]:.3f}°, J6={joints[5]:.3f}°")
                    print(f"   좌표계: X={pose[0]:.2f}mm, Y={pose[1]:.2f}mm, Z={pose[2]:.2f}mm")
                    print(f"           Rx={pose[3]:.2f}°, Ry={pose[4]:.2f}°, Rz={pose[5]:.2f}°")
                    print(f"   로봇 모드: {mode}")
                    print(f"   석션/그리퍼: ToolDO1={tooldo1}, ToolDO2={tooldo2}")
                    print("-" * 70)
                else:
                    print("\n⚠️  피드백 데이터 유효성 검사 실패")
            else:
                print("\n⚠️  피드백 데이터를 읽을 수 없습니다.")
        except Exception as e:
            print(f"\n⚠️  위치 조회 오류: {e}")
            import traceback
            traceback.print_exc()
    
    def show_help(self):
        """도움말 표시"""
        print("\n" + "=" * 70)
        print("📖 사용 가능한 명령:")
        print("=" * 70)
        print("  r  - 로깅 시작 (Record)")
        print("  s  - 로깅 중지 및 저장 (Stop)")
        print("  p  - 현재 위치 확인 (Position)")
        print("  h  - 도움말 (Help)")
        print("  q  - 프로그램 종료 (Quit)")
        print("=" * 70)
    
    def run_interactive_mode(self):
        """대화형 모드 실행"""
        print("\n" + "=" * 70)
        print("🎓 수동 티칭 로깅 모드")
        print("=" * 70)
        print("💡 로봇을 수동으로 움직일 수 있는 상태인지 확인하세요.")
        print("   (Enable 해제, 브레이크 해제, 또는 드래그 모드)")
        print()
        
        self.show_help()
        
        print("\n명령을 기다리는 중... (아무 키나 누르세요)")
        
        while True:
            # 키 입력 대기 (논블로킹)
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                
                if key == 'r':
                    # 로깅 시작
                    if not self.logging_active:
                        session_name = f"Session_{self.teaching_session + 1}"
                        self.start_logging(session_name)
                    else:
                        print("\n⚠️  이미 로깅이 진행 중입니다.")
                
                elif key == 's':
                    # 로깅 중지
                    if self.logging_active:
                        self.stop_logging()
                    else:
                        print("\n⚠️  로깅이 진행 중이 아닙니다.")
                
                elif key == 'p':
                    # 현재 위치 표시
                    self.display_current_position()
                
                elif key == 'h':
                    # 도움말
                    self.show_help()
                
                elif key == 'q':
                    # 종료
                    if self.logging_active:
                        print("\n⚠️  로깅을 먼저 중지해주세요. (s 키 입력)")
                    else:
                        print("\n👋 프로그램을 종료합니다.")
                        break
                
                else:
                    print(f"\n⚠️  알 수 없는 명령: {key}")
                    print("   'h' 키를 눌러 도움말을 확인하세요.")
            
            # 로깅 중일 때 진행 상황 표시
            if self.logging_active:
                if len(self.log_data) % 20 == 0:  # 1초마다 (20Hz)
                    print(f"📝 로깅 중... ({len(self.log_data)}개 데이터 수집)", end='\r')
            
            sleep(0.1)


def main():
    print("\n🤖 수동 티칭 로깅 프로그램")
    print()
    print("이 프로그램은 로봇을 수동으로 움직이는 동안")
    print("실시간으로 관절값과 좌표값을 기록합니다.")
    print()
    print("=" * 70)
    
    try:
        logger = ManualTeachingLogger()
        
        # 연결
        if not logger.connect():
            print("\n로봇 연결 실패. 프로그램을 종료합니다.")
            return
        
        print("\n⚠️  주의사항:")
        print("  1. 로봇이 수동으로 움직일 수 있는 상태인지 확인하세요.")
        print("     - DobotStudio Pro에서 Disable 상태")
        print("     - 또는 드래그 티칭 모드 활성화")
        print("  2. 로봇 주변에 장애물이 없는지 확인하세요.")
        print("  3. 안전을 위해 천천히 움직이세요.")
        
        input("\n준비되었으면 Enter 키를 누르세요...")
        
        # 대화형 모드 시작
        logger.run_interactive_mode()
        
    except KeyboardInterrupt:
        print("\n\n사용자가 중단했습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
