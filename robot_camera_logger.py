"""
로봇 + 카메라 통합 로거 (HIKRobot MV-CE050-30UC + Dobot)
카메라 프레임과 로봇 좌표를 완벽하게 동기화하여 저장

사용 방법:
1. python robot_camera_logger.py
2. 'r' 키로 로깅 시작
3. 로봇을 수동으로 움직임
4. 's' 키로 저장

저장 형식:
- CSV: 로봇 좌표 데이터
- NPY: 이미지 + 좌표 동기화 데이터 (VLA 학습용)
- 개별 JPG: 각 프레임 이미지
"""

from dobot_api import DobotApiDashboard, DobotApiFeedBack
from time import sleep
import threading
import time
from datetime import datetime
import csv
import os
import sys
import numpy as np
import cv2

# HIKRobot SDK
try:
    from MvCameraControl_class import *
except ImportError:
    print("❌ MvCameraControl_class.py를 찾을 수 없습니다.")
    print("   C:\\Program Files (x86)\\MVS\\Development\\Samples\\Python\\")
    print("   에서 MvCameraControl_class.py를 프로젝트 폴더로 복사하세요.")
    sys.exit(1)

# Windows 키 입력
try:
    import msvcrt
except ImportError:
    print("이 프로그램은 Windows에서만 작동합니다.")
    sys.exit(1)


class RobotCameraLogger:
    def __init__(self, robot_ip="192.168.5.1"):
        self.robot_ip = robot_ip
        self.dashboard = None
        self.feedback = None
        
        # 로깅 관련
        self.logging_active = False
        self.log_data = []
        self.sync_data = []  # 이미지 + 좌표 동기화 데이터
        self.log_thread = None
        self.session_id = 0
        
        # 카메라 관련
        self.camera = None
        self.camera_active = False
        self.frame_buffer = None
        self.frame_lock = threading.Lock()
        
    def init_camera(self):
        """HIKRobot 카메라 초기화"""
        print("\n" + "=" * 70)
        print("📷 HIKRobot 카메라 초기화 중...")
        print("=" * 70)
        
        try:
            # 장치 목록 가져오기
            deviceList = MV_CC_DEVICE_INFO_LIST()
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
            
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0:
                print(f"❌ 카메라를 찾을 수 없습니다. (에러 코드: {ret})")
                return False
            
            if deviceList.nDeviceNum == 0:
                print("❌ 연결된 HIKRobot 카메라가 없습니다.")
                return False
            
            print(f"✅ {deviceList.nDeviceNum}개의 카메라 발견")
            
            # 첫 번째 카메라 사용
            self.camera = MvCamera()
            ret = self.camera.MV_CC_CreateHandle(deviceList.pDeviceInfo[0])
            
            if ret != 0:
                print(f"❌ 카메라 핸들 생성 실패 (에러 코드: {ret})")
                return False
            
            # 카메라 열기
            ret = self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                print(f"❌ 카메라 열기 실패 (에러 코드: {ret})")
                return False
            
            # 트리거 모드 OFF
            ret = self.camera.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                print(f"⚠️  트리거 모드 설정 경고")
            
            # 프레임레이트 설정 (USB 2.0 대역폭 고려)
            print("⚙️  프레임레이트 설정 중... (USB 2.0 대역폭 제약)")
            ret = self.camera.MV_CC_SetEnumValue("AcquisitionFrameRateEnable", 1)
            ret = self.camera.MV_CC_SetFloatValue("AcquisitionFrameRate", 10.0)  # 10 FPS로 제한
            
            print("   → 프레임레이트: 10 FPS (대역폭 절약)")
            
            # 스트리밍 시작
            ret = self.camera.MV_CC_StartGrabbing()
            if ret != 0:
                print(f"❌ 스트리밍 시작 실패 (에러 코드: {ret})")
                return False
            
            print("✅ 카메라 초기화 완료")
            self.camera_active = True
            
            # 카메라 캡처 스레드 시작
            camera_thread = threading.Thread(target=self._camera_worker)
            camera_thread.daemon = True
            camera_thread.start()
            
            return True
            
        except Exception as e:
            print(f"❌ 카메라 초기화 오류: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _camera_worker(self):
        """카메라 프레임 캡처 스레드 (백그라운드)"""
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        
        while self.camera_active:
            try:
                # 프레임 가져오기 (1000ms 타임아웃)
                pData = (c_ubyte * (2048 * 2048 * 3))()
                ret = self.camera.MV_CC_GetOneFrameTimeout(pData, 2048 * 2048 * 3, stFrameInfo, 1000)
                
                if ret == 0:
                    # NumPy 배열로 변환
                    image_data = np.frombuffer(pData, dtype=np.uint8, count=stFrameInfo.nFrameLen)
                    
                    # Bayer 또는 RGB 형식에 따라 처리
                    if stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                        # Bayer → RGB 변환
                        image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                        image = cv2.cvtColor(image, cv2.COLOR_BayerRG2RGB)
                    else:
                        # RGB 또는 Mono
                        image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, -1))
                    
                    # 640x480으로 리사이즈 (용량 절약)
                    image = cv2.resize(image, (640, 480))
                    
                    # BGR ↔ RGB 변환 (색상 반전 수정)
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    
                    # 버퍼에 저장 (최신 프레임만 유지)
                    with self.frame_lock:
                        self.frame_buffer = image.copy()
                
                sleep(0.01)  # CPU 절약
                
            except Exception as e:
                print(f"⚠️  카메라 캡처 오류: {e}")
                sleep(0.1)
    
    def get_latest_frame(self):
        """최신 카메라 프레임 가져오기"""
        with self.frame_lock:
            if self.frame_buffer is not None:
                return self.frame_buffer.copy()
            return None
    
    def connect_robot(self):
        """로봇 연결"""
        print("\n" + "=" * 70)
        print("🤖 로봇 연결 중...")
        print("=" * 70)
        
        try:
            self.dashboard = DobotApiDashboard(self.robot_ip, 29999)
            self.feedback = DobotApiFeedBack(self.robot_ip, 30004)
            
            print("✅ 로봇 연결 완료")
            
            # 피드백 스레드는 로깅 시작 시에만 활성화
            return True
            
        except Exception as e:
            print(f"❌ 로봇 연결 실패: {e}")
            return False
    
    def start_logging(self):
        """로깅 시작"""
        if self.logging_active:
            print("\n⚠️  이미 로깅이 진행 중입니다.")
            return
        
        self.session_id += 1
        session_name = f"Session_{self.session_id}"
        
        self.logging_active = True
        self.log_data = []
        self.sync_data = []
        
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
        """로깅 종료 및 저장"""
        if not self.logging_active:
            print("\n⚠️  로깅이 진행 중이 아닙니다.")
            return
        
        self.logging_active = False
        if self.log_thread:
            self.log_thread.join(timeout=2.0)
        
        print("\n" + "=" * 70)
        print(f"📝 로깅 종료")
        print(f"   로그 데이터: {len(self.log_data)}개")
        print(f"   동기화 데이터: {len(self.sync_data)}개")
        print("=" * 70)
        
        # 저장
        if self.log_data:
            self.save_data()
    
    def _logging_worker(self, session_name):
        """로깅 작업 스레드 (로봇 + 카메라 동기화)"""
        start_time = time.time()
        
        while self.logging_active:
            try:
                # 1. 카메라 프레임 가져오기
                frame = self.get_latest_frame()
                
                # 2. 로봇 피드백 데이터 수집
                data = self.feedback.feedBackData()
                
                if data is not None and len(data) > 0:
                    if hex((data['TestValue'][0])) == '0x123456789abcdef':
                        current_time = time.time() - start_time
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        
                        # 관절 각도
                        joints = data['QActual'][0]
                        j1, j2, j3, j4, j5, j6 = joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]
                        
                        # 직교 좌표
                        pose = data['ToolVectorActual'][0]
                        x, y, z, rx, ry, rz = pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
                        
                        # 로봇 모드
                        robot_mode = data['RobotMode'][0]
                        
                        # ToolDO 상태
                        try:
                            tooldo1_result = self.dashboard.GetToolDO(1)
                            tooldo1 = 1 if ',{1},' in tooldo1_result else 0
                        except:
                            tooldo1 = 0
                        
                        try:
                            tooldo2_result = self.dashboard.GetToolDO(2)
                            tooldo2 = 1 if ',{1},' in tooldo2_result else 0
                        except:
                            tooldo2 = 0
                        
                        # CSV용 로그 데이터
                        log_entry = {
                            'session_name': session_name,
                            'timestamp': timestamp,
                            'elapsed_time': f"{current_time:.3f}",
                            'j1': f"{j1:.4f}", 'j2': f"{j2:.4f}", 'j3': f"{j3:.4f}",
                            'j4': f"{j4:.4f}", 'j5': f"{j5:.4f}", 'j6': f"{j6:.4f}",
                            'x': f"{x:.4f}", 'y': f"{y:.4f}", 'z': f"{z:.4f}",
                            'rx': f"{rx:.4f}", 'ry': f"{ry:.4f}", 'rz': f"{rz:.4f}",
                            'robot_mode': robot_mode,
                            'tooldo1': tooldo1,
                            'tooldo2': tooldo2
                        }
                        
                        self.log_data.append(log_entry)
                        
                        # 🆕 동기화 데이터 (이미지 + 좌표) - VLA 학습용!
                        if frame is not None:
                            sync_entry = {
                                'image': frame.copy(),  # 카메라 프레임
                                'joints': np.array([j1, j2, j3, j4, j5, j6], dtype=np.float32),
                                'pose': np.array([x, y, z, rx, ry, rz], dtype=np.float32),
                                'gripper': np.array([tooldo1, tooldo2], dtype=np.int32),
                                'timestamp': current_time
                            }
                            self.sync_data.append(sync_entry)
                
                sleep(0.1)  # 10Hz 샘플링 (카메라 프레임레이트와 맞춤)
                
            except Exception as e:
                print(f"⚠️  로깅 오류: {e}")
                import traceback
                traceback.print_exc()
                break
    
    def save_data(self):
        """데이터 저장 (CSV + NPY + 이미지)"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_id = f"robot_camera_{timestamp}"
        
        # 폴더 생성
        log_dir = "logs"
        frames_dir = os.path.join(log_dir, f"frames_{session_id}")
        os.makedirs(log_dir, exist_ok=True)
        os.makedirs(frames_dir, exist_ok=True)
        
        # 1. CSV 저장 (로봇 좌표만)
        csv_file = os.path.join(log_dir, f"{session_id}.csv")
        with open(csv_file, 'w', newline='', encoding='utf-8-sig') as f:
            fieldnames = [
                'session_name', 'timestamp', 'elapsed_time',
                'j1', 'j2', 'j3', 'j4', 'j5', 'j6',
                'x', 'y', 'z', 'rx', 'ry', 'rz',
                'robot_mode', 'tooldo1', 'tooldo2'
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_data)
        
        print(f"\n💾 CSV 저장: {csv_file}")
        
        # 2. NPY 저장 (동기화 데이터 - VLA 학습용!)
        npy_file = os.path.join(log_dir, f"{session_id}_sync.npy")
        np.save(npy_file, self.sync_data, allow_pickle=True)
        
        print(f"💾 NPY 저장: {npy_file}")
        print(f"   → VLA 학습용 동기화 데이터 (이미지 + 좌표)")
        
        # 3. 개별 이미지 저장 (JPG)
        print(f"\n📸 이미지 저장 중... ({len(self.sync_data)}장)")
        for idx, entry in enumerate(self.sync_data):
            frame_file = os.path.join(frames_dir, f"frame_{idx:05d}.jpg")
            cv2.imwrite(frame_file, entry['image'], [cv2.IMWRITE_JPEG_QUALITY, 90])
        
        print(f"✅ 이미지 저장 완료: {frames_dir}/")
        
        print("\n" + "=" * 70)
        print("✅ 모든 데이터 저장 완료!")
        print("=" * 70)
        print(f"\n저장된 파일:")
        print(f"  1. {csv_file} (로봇 좌표)")
        print(f"  2. {npy_file} (이미지+좌표 동기화)")
        print(f"  3. {frames_dir}/ (개별 이미지)")
        print(f"\n💡 VLA 학습 시 {npy_file}를 로드하세요!")
    
    def show_help(self):
        """도움말"""
        print("\n" + "=" * 70)
        print("📖 사용 가능한 명령:")
        print("=" * 70)
        print("  r  - 로깅 시작 (Record)")
        print("  s  - 로깅 중지 및 저장 (Stop)")
        print("  h  - 도움말 (Help)")
        print("  q  - 프로그램 종료 (Quit)")
        print("=" * 70)
    
    def run(self):
        """메인 루프"""
        print("\n" + "=" * 70)
        print("🤖📷 로봇 + 카메라 통합 로거")
        print("=" * 70)
        print("💡 로봇을 수동으로 움직이며 카메라 영상과 함께 기록합니다.")
        print("=" * 70)
        
        self.show_help()
        
        print("\n명령을 기다리는 중... (아무 키나 누르세요)")
        
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                
                if key == 'r':
                    if not self.logging_active:
                        self.start_logging()
                    else:
                        print("\n⚠️  이미 로깅이 진행 중입니다.")
                
                elif key == 's':
                    if self.logging_active:
                        self.stop_logging()
                    else:
                        print("\n⚠️  로깅이 진행 중이 아닙니다.")
                
                elif key == 'h':
                    self.show_help()
                
                elif key == 'q':
                    if self.logging_active:
                        print("\n⚠️  로깅을 먼저 중지해주세요. (s 키)")
                    else:
                        print("\n👋 프로그램을 종료합니다.")
                        break
                
                else:
                    print(f"\n⚠️  알 수 없는 명령: {key}")
            
            # 진행 상황 표시
            if self.logging_active:
                if len(self.log_data) % 10 == 0:
                    print(f"📝 로깅 중... ({len(self.log_data)}개 데이터, {len(self.sync_data)}개 프레임)", end='\r')
            
            sleep(0.1)
    
    def cleanup(self):
        """리소스 정리"""
        if self.camera:
            try:
                self.camera_active = False
                sleep(0.5)
                self.camera.MV_CC_StopGrabbing()
                self.camera.MV_CC_CloseDevice()
                self.camera.MV_CC_DestroyHandle()
                print("📷 카메라 리소스 해제 완료")
            except:
                pass


def main():
    print("\n" + "=" * 70)
    print("🤖📷 로봇 + 카메라 통합 로거 (HIKRobot + Dobot)")
    print("=" * 70)
    
    logger = RobotCameraLogger()
    
    try:
        # 1. 카메라 초기화
        if not logger.init_camera():
            print("\n❌ 카메라 초기화 실패. 프로그램을 종료합니다.")
            return
        
        # 2. 로봇 연결
        if not logger.connect_robot():
            print("\n❌ 로봇 연결 실패. 프로그램을 종료합니다.")
            return
        
        print("\n⚠️  주의사항:")
        print("  1. 로봇이 수동으로 움직일 수 있는 상태인지 확인하세요.")
        print("  2. 카메라가 로봇 작업 영역을 잘 볼 수 있는지 확인하세요.")
        print("  3. 프레임레이트는 10 FPS로 제한됩니다 (USB 2.0 대역폭)")
        
        input("\n준비되었으면 Enter 키를 누르세요...")
        
        # 3. 메인 루프
        logger.run()
    
    except KeyboardInterrupt:
        print("\n\n사용자가 중단했습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        logger.cleanup()


if __name__ == "__main__":
    main()
