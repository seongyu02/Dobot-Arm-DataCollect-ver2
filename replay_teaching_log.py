"""
티칭 로그 재생 프로그램
수동 티칭으로 기록한 경로를 로봇이 재현합니다.

사용 방법:
1. 프로그램 실행
2. 로그 파일 선택
3. 관절각 모드 또는 직교좌표 모드 선택
4. 재생 시작
"""

from dobot_api import DobotApiDashboard, DobotApiFeedBack
from time import sleep
import csv
import os
import sys
from datetime import datetime

class TeachingLogReplayer:
    def __init__(self, ip="192.168.5.1"):
        self.ip = ip
        self.dashboard = None
        self.feedback = None
        self.log_data = []
        
    def connect(self):
        """로봇 연결"""
        print("=" * 70)
        print("🤖 로봇 연결 중...")
        print("=" * 70)
        
        try:
            self.dashboard = DobotApiDashboard(self.ip, 29999)
            self.feedback = DobotApiFeedBack(self.ip, 30004)
            
            sleep(0.5)
            print("✅ 연결 완료")
            return True
            
        except Exception as e:
            print(f"❌ 연결 실패: {e}")
            return False
    
    def enable_robot(self):
        """로봇 활성화"""
        print("\n🔧 로봇 활성화 중...")
        
        try:
            # 에러 클리어
            self.dashboard.ClearError()
            sleep(0.5)
            
            # Enable
            result = self.dashboard.EnableRobot()
            print(f"   EnableRobot 결과: {result}")
            sleep(1)
            
            # 상태 확인
            for i in range(10):
                data = self.feedback.feedBackData()
                if data is not None and len(data) > 0:
                    mode = data['RobotMode'][0]
                    if mode == 5:  # ROBOT_MODE_ENABLE
                        print(f"✅ 로봇 활성화 성공 (RobotMode: {mode})")
                        return True
                    elif mode == 7:  # ROBOT_MODE_RUNNING
                        print(f"✅ 로봇 준비 완료 (RobotMode: {mode})")
                        return True
                    print(f"   대기 중... (RobotMode: {mode})")
                sleep(0.5)
            
            print("⚠️ 로봇 활성화 시간 초과")
            return False
            
        except Exception as e:
            print(f"❌ 로봇 활성화 실패: {e}")
            return False
    
    def _wait_for_robot_stop(self, timeout=10.0):
        """로봇이 정지할 때까지 대기"""
        import time
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                data = self.feedback.feedBackData()
                if data is not None and len(data) > 0:
                    # RunningStatus 확인: 0=정지, 1=실행중
                    running_status = data['RunningStatus'][0]
                    if running_status == 0:
                        # 추가로 짧은 대기 (진동 감쇠)
                        sleep(0.05)
                        return True
            except:
                pass
            sleep(0.01)
        
        # 타임아웃 시에도 계속 진행 (경고만 출력)
        return False
    
    def list_log_files(self):
        """로그 폴더의 파일 목록 표시"""
        log_dir = "logs"
        
        if not os.path.exists(log_dir):
            print(f"⚠️ 로그 폴더가 없습니다: {log_dir}")
            return []
        
        files = [f for f in os.listdir(log_dir) if f.endswith('.csv')]
        
        if not files:
            print(f"⚠️ 로그 파일이 없습니다.")
            return []
        
        # 최신 파일 순으로 정렬
        files.sort(reverse=True)
        
        print("\n" + "=" * 70)
        print("📁 사용 가능한 로그 파일:")
        print("=" * 70)
        
        for idx, filename in enumerate(files, 1):
            filepath = os.path.join(log_dir, filename)
            size = os.path.getsize(filepath)
            print(f"  {idx}. {filename} ({size:,} bytes)")
        
        print("=" * 70)
        
        return files
    
    def load_log_file(self, filepath):
        """로그 파일 읽기"""
        print(f"\n📂 로그 파일 로딩: {filepath}")
        
        try:
            self.log_data = []
            
            with open(filepath, 'r', encoding='utf-8-sig') as csvfile:
                reader = csv.DictReader(csvfile)
                
                for row in reader:
                    entry = {
                        'session_name': row['session_name'],
                        'timestamp': row['timestamp'],
                        'elapsed_time': float(row['elapsed_time']),
                        'j1': float(row['j1']),
                        'j2': float(row['j2']),
                        'j3': float(row['j3']),
                        'j4': float(row['j4']),
                        'j5': float(row['j5']),
                        'j6': float(row['j6']),
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'z': float(row['z']),
                        'rx': float(row['rx']),
                        'ry': float(row['ry']),
                        'rz': float(row['rz']),
                        'robot_mode': int(row['robot_mode'])
                    }
                    
                    # ToolDO 상태 (석션/그리퍼, 옵션)
                    entry['tooldo1'] = int(row.get('tooldo1', 0))
                    entry['tooldo2'] = int(row.get('tooldo2', 0))
                    
                    # 일반 DO 상태 (하위 호환성, 옵션)
                    entry['do1'] = int(row.get('do1', 0))
                    entry['do2'] = int(row.get('do2', 0))
                    
                    self.log_data.append(entry)
            
            print(f"✅ 로그 로딩 완료: {len(self.log_data)}개 데이터 포인트")
            
            # 요약 정보 출력
            if self.log_data:
                duration = self.log_data[-1]['elapsed_time']
                print(f"\n📊 로그 정보:")
                print(f"   세션명: {self.log_data[0]['session_name']}")
                print(f"   총 시간: {duration:.2f}초")
                print(f"   데이터 포인트: {len(self.log_data)}개")
                print(f"   평균 샘플링 속도: {len(self.log_data)/duration:.1f}Hz")
            
            return True
            
        except Exception as e:
            print(f"❌ 로그 로딩 실패: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def replay_joint_mode(self, speed_ratio=100, sample_skip=1):
        """관절각 모드로 재생"""
        print("\n" + "=" * 70)
        print("🎬 관절각 모드로 재생 시작")
        print("=" * 70)
        print(f"⚙️  속도 비율: {speed_ratio}%")
        print(f"⚙️  샘플 건너뛰기: {sample_skip} (1 = 모든 포인트)")
        print("=" * 70)
        
        total_points = len(self.log_data)
        replay_points = list(range(0, total_points, sample_skip))
        
        print(f"\n총 {len(replay_points)}개 포인트를 재생합니다.")
        print("Ctrl+C를 눌러 중단할 수 있습니다.\n")
        
        try:
            for idx, data_idx in enumerate(replay_points):
                point = self.log_data[data_idx]
                
                # MovJ 메서드 직접 호출 (관절각 모드, coordinateMode=1)
                result = self.dashboard.MovJ(
                    point['j1'],
                    point['j2'],
                    point['j3'],
                    point['j4'],
                    point['j5'],
                    point['j6'],
                    1,  # coordinateMode=1 (관절각)
                    v=speed_ratio  # 속도
                )
                
                # 로봇이 목표 위치에 도달할 때까지 대기 (피드백으로 정지 확인)
                self._wait_for_robot_stop(timeout=10.0)
                
                # 그리퍼/석션 상태 재현 (ToolDOInstant - 즉시 실행)
                if point.get('tooldo1') is not None:
                    self.dashboard.ToolDOInstant(1, point['tooldo1'])
                if point.get('tooldo2') is not None:
                    self.dashboard.ToolDOInstant(2, point['tooldo2'])
                
                # 진행 상황 표시
                progress = (idx + 1) / len(replay_points) * 100
                tooldo_status = f"ToolDO:[{point.get('tooldo1','-')},{point.get('tooldo2','-')}]"
                print(f"[{progress:5.1f}%] {idx+1}/{len(replay_points)} - "
                      f"J1={point['j1']:7.2f}°, J2={point['j2']:7.2f}°, J3={point['j3']:7.2f}°, "
                      f"J4={point['j4']:7.2f}°, J5={point['j5']:7.2f}°, J6={point['j6']:7.2f}° {tooldo_status}")
                
                # 에러 체크
                if "Not Tcp" in result:
                    print(f"⚠️  경고: TCP 모드가 아닙니다. - {result}")
                    break
                
                # 짧은 대기
                sleep(0.02)
            
            print("\n" + "=" * 70)
            print("✅ 재생 완료!")
            print("=" * 70)
            
        except KeyboardInterrupt:
            print("\n\n⚠️ 사용자가 재생을 중단했습니다.")
        except Exception as e:
            print(f"\n❌ 재생 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
    
    def replay_cartesian_mode(self, speed_ratio=100, sample_skip=1):
        """직교좌표 모드로 재생"""
        print("\n" + "=" * 70)
        print("🎬 직교좌표 모드로 재생 시작")
        print("=" * 70)
        print(f"⚙️  속도 비율: {speed_ratio}%")
        print(f"⚙️  샘플 건너뛰기: {sample_skip} (1 = 모든 포인트)")
        print("=" * 70)
        
        total_points = len(self.log_data)
        replay_points = list(range(0, total_points, sample_skip))
        
        print(f"\n총 {len(replay_points)}개 포인트를 재생합니다.")
        print("Ctrl+C를 눌러 중단할 수 있습니다.\n")
        
        try:
            for idx, data_idx in enumerate(replay_points):
                point = self.log_data[data_idx]
                
                # MovJ 메서드로 직교좌표 전송 (coordinateMode=0)
                result = self.dashboard.MovJ(
                    point['x'],
                    point['y'],
                    point['z'],
                    point['rx'],
                    point['ry'],
                    point['rz'],
                    0,  # coordinateMode=0 (직교좌표)
                    v=speed_ratio  # 속도
                )
                
                # 로봇이 목표 위치에 도달할 때까지 대기 (피드백으로 정지 확인)
                self._wait_for_robot_stop(timeout=10.0)
                
                # 그리퍼/석션 상태 재현 (ToolDOInstant - 즉시 실행)
                if point.get('tooldo1') is not None:
                    self.dashboard.ToolDOInstant(1, point['tooldo1'])
                if point.get('tooldo2') is not None:
                    self.dashboard.ToolDOInstant(2, point['tooldo2'])
                
                # 진행 상황 표시
                progress = (idx + 1) / len(replay_points) * 100
                do_status = f"DO:[{point.get('do1','-')},{point.get('do2','-')},{point.get('do3','-')},{point.get('do4','-')}]"
                print(f"[{progress:5.1f}%] {idx+1}/{len(replay_points)} - "
                      f"X={point['x']:7.2f}, Y={point['y']:7.2f}, Z={point['z']:7.2f}, "
                      f"Rx={point['rx']:7.2f}°, Ry={point['ry']:7.2f}°, Rz={point['rz']:7.2f}° {do_status}")
                
                # 에러 체크
                if "Not Tcp" in result:
                    print(f"⚠️  경고: TCP 모드가 아닙니다. - {result}")
                    break
                
                # 짧은 대기
                sleep(0.02)
            
            print("\n" + "=" * 70)
            print("✅ 재생 완료!")
            print("=" * 70)
            
        except KeyboardInterrupt:
            print("\n\n⚠️ 사용자가 재생을 중단했습니다.")
        except Exception as e:
            print(f"\n❌ 재생 중 오류 발생: {e}")
            import traceback
            traceback.print_exc()
    
    def run_interactive_mode(self):
        """대화형 모드 실행"""
        print("\n🎬 티칭 로그 재생 프로그램")
        print()
        
        # 1. 로그 파일 선택
        files = self.list_log_files()
        
        if not files:
            return
        
        while True:
            try:
                choice = input("\n📝 재생할 파일 번호를 입력하세요 (0=취소): ")
                
                if choice == '0':
                    print("취소되었습니다.")
                    return
                
                idx = int(choice) - 1
                
                if 0 <= idx < len(files):
                    filepath = os.path.join("logs", files[idx])
                    break
                else:
                    print("⚠️ 잘못된 번호입니다.")
            except ValueError:
                print("⚠️ 숫자를 입력하세요.")
        
        # 2. 로그 파일 로딩
        if not self.load_log_file(filepath):
            return
        
        # 3. 재생 모드 선택
        print("\n" + "=" * 70)
        print("🎯 재생 모드 선택:")
        print("=" * 70)
        print("  1. 관절각 모드 (MovJ) - 원래 관절 각도를 재현")
        print("  2. 직교좌표 모드 (MovL) - 원래 좌표 위치를 재현")
        print("=" * 70)
        
        while True:
            mode_choice = input("\n재생 모드를 선택하세요 (1 또는 2): ")
            
            if mode_choice in ['1', '2']:
                break
            else:
                print("⚠️ 1 또는 2를 입력하세요.")
        
        # 4. 속도 설정
        print("\n" + "=" * 70)
        print("⚡ 속도 설정:")
        print("=" * 70)
        print("  - 권장: 50-100% (너무 빠르면 위험할 수 있습니다)")
        print("=" * 70)
        
        while True:
            try:
                speed_input = input("\n속도 비율을 입력하세요 (1-100%, Enter=50%): ")
                
                if speed_input == '':
                    speed_ratio = 50
                    break
                
                speed_ratio = int(speed_input)
                
                if 1 <= speed_ratio <= 100:
                    break
                else:
                    print("⚠️ 1-100 사이의 값을 입력하세요.")
            except ValueError:
                print("⚠️ 숫자를 입력하세요.")
        
        # 5. 샘플링 간격 설정
        print("\n" + "=" * 70)
        print("📊 샘플링 간격 설정:")
        print("=" * 70)
        print("  - 1: 모든 포인트 재생 (가장 정확하지만 느림)")
        print("  - 5: 5개 중 1개 재생 (빠르지만 부드럽지 않을 수 있음)")
        print("  - 10: 10개 중 1개 재생 (매우 빠름)")
        print("=" * 70)
        
        while True:
            try:
                skip_input = input("\n샘플 건너뛰기 값을 입력하세요 (1-20, Enter=5): ")
                
                if skip_input == '':
                    sample_skip = 5
                    break
                
                sample_skip = int(skip_input)
                
                if 1 <= sample_skip <= 20:
                    break
                else:
                    print("⚠️ 1-20 사이의 값을 입력하세요.")
            except ValueError:
                print("⚠️ 숫자를 입력하세요.")
        
        # 6. 최종 확인
        print("\n" + "=" * 70)
        print("📋 재생 설정 확인:")
        print("=" * 70)
        print(f"  파일: {files[idx]}")
        print(f"  모드: {'관절각 모드 (MovJ)' if mode_choice == '1' else '직교좌표 모드 (MovL)'}")
        print(f"  속도: {speed_ratio}%")
        print(f"  샘플 간격: {sample_skip}")
        print(f"  재생 포인트: 약 {len(self.log_data) // sample_skip}개")
        print("=" * 70)
        
        confirm = input("\n⚠️  재생을 시작하시겠습니까? (y/n): ")
        
        if confirm.lower() != 'y':
            print("취소되었습니다.")
            return
        
        # 7. 로봇 연결 및 활성화
        if not self.connect():
            return
        
        if not self.enable_robot():
            return
        
        # 8. 재생 시작
        if mode_choice == '1':
            self.replay_joint_mode(speed_ratio, sample_skip)
        else:
            self.replay_cartesian_mode(speed_ratio, sample_skip)


def main():
    print("\n" + "=" * 70)
    print("🎬 티칭 로그 재생 프로그램")
    print("=" * 70)
    print()
    print("이 프로그램은 수동 티칭으로 기록한 경로를 로봇이 재현합니다.")
    print()
    print("⚠️  주의사항:")
    print("  1. 로봇 주변에 장애물이 없는지 확인하세요.")
    print("  2. 비상 정지 버튼을 언제든지 누를 수 있도록 준비하세요.")
    print("  3. 처음에는 낮은 속도(30-50%)로 테스트하세요.")
    print("=" * 70)
    
    input("\n준비되었으면 Enter 키를 누르세요...")
    
    try:
        replayer = TeachingLogReplayer()
        replayer.run_interactive_mode()
        
    except KeyboardInterrupt:
        print("\n\n사용자가 중단했습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
