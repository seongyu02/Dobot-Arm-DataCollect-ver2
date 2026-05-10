"""
HIKRobot 카메라용 캘리브레이션 프로그램
체커보드 패턴을 사용하여 카메라 내부 파라미터 계산

사용 방법:
1. 체커보드 패턴을 출력하거나 준비 (기본: 9x6, 25mm)
2. python camera_calibration_hikrobot.py
3. Space 키를 눌러 다양한 각도에서 이미지 캡처 (최소 15장)
4. 'c' 키를 눌러 캘리브레이션 실행
"""

import sys
import os
import cv2
import numpy as np
from ctypes import *
import time
from datetime import datetime

# HIKRobot Runtime DLL 경로 추가
if hasattr(os, 'add_dll_directory'):
    dll_path = r"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64"
    if os.path.exists(dll_path):
        os.add_dll_directory(dll_path)

# HIKRobot SDK
try:
    from MvImport.MvCameraControl_class import *
except ImportError:
    print("\n" + "=" * 70)
    print("오류: HIKRobot SDK를 찾을 수 없습니다.")
    print("=" * 70)
    print("\n해결 방법:")
    print("1. MVS가 설치되어 있는지 확인")
    print("2. MvImport 폴더가 프로젝트에 있는지 확인")
    print("=" * 70)
    sys.exit(1)


class HikRobotCalibration:
    def __init__(self, chessboard_size=(9, 6), square_size=25.0):
        """
        Args:
            chessboard_size: (가로 코너 수, 세로 코너 수)
            square_size: 체커보드 한 칸의 크기 (mm)
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        
        # 캘리브레이션 데이터
        self.objpoints = []  # 3D 포인트 (실제 세계 좌표)
        self.imgpoints = []  # 2D 포인트 (이미지 좌표)
        self.captured_images = []
        
        # 카메라 관련
        self.camera = None
        self.deviceList = None
        
        # 캘리브레이션 결과
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.rms_error = None
        
    def init_camera(self):
        """HIKRobot 카메라 초기화"""
        print("\n" + "=" * 70)
        print("카메라 초기화 중...")
        print("=" * 70)
        
        try:
            # SDK 초기화
            ret = MvCamera.MV_CC_Initialize()
            if ret != 0:
                print(f"오류: SDK 초기화 실패 (0x{ret:x})")
                return False
            
            # 카메라 검색
            self.deviceList = MV_CC_DEVICE_INFO_LIST()
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
            
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, self.deviceList)
            if ret != 0:
                print(f"오류: 카메라 검색 실패 (0x{ret:x})")
                return False
            
            if self.deviceList.nDeviceNum == 0:
                print("오류: 연결된 카메라가 없습니다.")
                print("  - USB 케이블을 확인하세요")
                print("  - MVS Client에서 카메라가 보이는지 확인하세요")
                return False
            
            print(f"카메라 발견: {self.deviceList.nDeviceNum}개")
            
            # 카메라 열기
            self.camera = MvCamera()
            stDeviceInfo = cast(self.deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
            
            ret = self.camera.MV_CC_CreateHandle(stDeviceInfo)
            if ret != 0:
                print(f"오류: 카메라 핸들 생성 실패 (0x{ret:x})")
                return False
            
            ret = self.camera.MV_CC_OpenDevice()
            if ret != 0:
                print(f"오류: 카메라 열기 실패 (0x{ret:x})")
                return False
            
            # 카메라 설정
            self.camera.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            self.camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True)
            self.camera.MV_CC_SetFloatValue("AcquisitionFrameRate", 10.0)
            
            # 밝기 설정 (노출 자동 조정)
            print("밝기 설정 중...")
            try:
                # 자동 노출 켜기
                self.camera.MV_CC_SetEnumValue("ExposureAuto", 2)  # 2 = Continuous (자동)
                print("  자동 노출: ON")
            except:
                print("  자동 노출 설정 실패")
            
            try:
                # 자동 게인 켜기
                self.camera.MV_CC_SetEnumValue("GainAuto", 2)  # 2 = Continuous (자동)
                print("  자동 게인: ON")
            except:
                print("  자동 게인 설정 실패")
            
            # 자동 조정이 안 되면 수동으로 밝게 설정
            try:
                # 노출 시간 증가 (마이크로초 단위)
                self.camera.MV_CC_SetFloatValue("ExposureTime", 20000)  # 20ms
                print("  노출 시간: 20000us (20ms)")
            except:
                pass
            
            try:
                # 게인 증가
                self.camera.MV_CC_SetFloatValue("Gain", 10.0)  # dB
                print("  게인: 10.0 dB")
            except:
                pass
            
            # 스트리밍 시작
            ret = self.camera.MV_CC_StartGrabbing()
            if ret != 0:
                print(f"오류: 스트리밍 시작 실패 (0x{ret:x})")
                return False
            
            print("카메라 초기화 완료!")
            return True
            
        except Exception as e:
            print(f"오류: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def close_camera(self):
        """카메라 종료"""
        if self.camera:
            try:
                self.camera.MV_CC_StopGrabbing()
                self.camera.MV_CC_CloseDevice()
                self.camera.MV_CC_DestroyHandle()
            except:
                pass
        
        try:
            MvCamera.MV_CC_Finalize()
        except:
            pass
    
    def get_frame(self):
        """HIKRobot 카메라에서 프레임 가져오기"""
        try:
            buffer_size = 2448 * 2048 * 3
            pData = (c_ubyte * buffer_size)()
            stFrameInfo = MV_FRAME_OUT_INFO_EX()
            memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
            
            ret = self.camera.MV_CC_GetOneFrameTimeout(pData, buffer_size, stFrameInfo, 1000)
            
            if ret == 0:
                image_data = np.frombuffer(pData, dtype=np.uint8, count=stFrameInfo.nFrameLen)
                
                # Bayer 형식 확인 및 변환
                if stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerRG2BGR)
                elif stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerGR2BGR)
                elif stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerGB2BGR)
                elif stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerBG2BGR)
                else:
                    # RGB 또는 Mono
                    if len(image_data) == stFrameInfo.nHeight * stFrameInfo.nWidth:
                        image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    else:
                        image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, -1))
                
                # 640x480으로 리사이즈
                image = cv2.resize(image, (640, 480))
                
                # BGR ↔ RGB 변환 (색상 반전 수정)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                return True, image
            else:
                return False, None
                
        except Exception as e:
            print(f"프레임 가져오기 오류: {e}")
            return False, None
    
    def capture_images(self):
        """체커보드 이미지 캡처"""
        print("\n" + "=" * 70)
        print("체커보드 이미지 캡처")
        print("=" * 70)
        print(f"\n체커보드 설정:")
        print(f"  크기: {self.chessboard_size[0]}x{self.chessboard_size[1]} (내부 코너)")
        print(f"  사각형 크기: {self.square_size}mm")
        print(f"\n조작 방법:")
        print(f"  Space   - 이미지 캡처 (최소 15장, 권장 20장 이상)")
        print(f"  c       - 캘리브레이션 시작")
        print(f"  +/-     - 밝기 조정 (노출 시간)")
        print(f"  a       - 자동 노출 ON/OFF")
        print(f"  q/ESC   - 종료")
        print(f"\n팁:")
        print(f"  - 다양한 각도에서 촬영 (정면, 좌우 기울임, 상하 기울임)")
        print(f"  - 다양한 거리에서 촬영 (가까이, 멀리)")
        print(f"  - 화면 전체 영역을 커버")
        print(f"  - 체커보드가 평평하고 흔들리지 않게")
        print("=" * 70)
        
        captured_count = 0
        current_exposure = 20000  # 초기 노출 시간 (us)
        auto_exposure = True
        
        while True:
            ret, frame = self.get_frame()
            
            if not ret or frame is None:
                time.sleep(0.01)
                continue
            
            # 체커보드 검출
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, 
                self.chessboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            
            # 화면에 표시할 프레임
            display_frame = frame.copy()
            
            # 체커보드가 검출되면 코너 그리기
            if found:
                cv2.drawChessboardCorners(display_frame, self.chessboard_size, corners, found)
                status_text = f"체커보드 검출됨 - Space로 캡처"
                status_color = (0, 255, 0)
            else:
                status_text = f"체커보드 찾는 중..."
                status_color = (0, 0, 255)
            
            # 상태 표시
            cv2.putText(display_frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(display_frame, f"캡처된 이미지: {captured_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 노출 정보 표시
            exposure_mode = "자동" if auto_exposure else f"수동 ({current_exposure:.0f}us)"
            cv2.putText(display_frame, f"노출: {exposure_mode}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            
            if captured_count >= 15:
                cv2.putText(display_frame, "충분한 이미지 수집 완료! 'c'로 캘리브레이션", (10, 450),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, f"최소 {15 - captured_count}장 더 필요", (10, 450),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
            cv2.imshow('HIKRobot Camera Calibration', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            # Space: 이미지 캡처
            if key == ord(' '):
                if found:
                    # 서브픽셀 정확도로 코너 개선
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    
                    # 3D 포인트 생성
                    objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
                    objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
                    objp *= self.square_size
                    
                    self.objpoints.append(objp)
                    self.imgpoints.append(corners_refined)
                    self.captured_images.append(frame.copy())
                    
                    captured_count += 1
                    print(f"이미지 캡처 완료! ({captured_count}장)")
                    
                    # 피드백 효과
                    feedback = display_frame.copy()
                    cv2.rectangle(feedback, (0, 0), (640, 480), (0, 255, 0), 10)
                    cv2.imshow('HIKRobot Camera Calibration', feedback)
                    cv2.waitKey(100)
                else:
                    print("경고: 체커보드를 찾을 수 없습니다!")
            
            # c: 캘리브레이션 시작
            elif key == ord('c'):
                if captured_count >= 15:
                    cv2.destroyAllWindows()
                    return True
                else:
                    print(f"최소 15장의 이미지가 필요합니다. (현재: {captured_count}장)")
            
            # +: 밝기 증가
            elif key == ord('+') or key == ord('='):
                auto_exposure = False
                self.camera.MV_CC_SetEnumValue("ExposureAuto", 0)  # 수동
                current_exposure = min(current_exposure * 1.5, 100000)  # 최대 100ms
                self.camera.MV_CC_SetFloatValue("ExposureTime", current_exposure)
                print(f"노출 시간 증가: {current_exposure:.0f}us")
            
            # -: 밝기 감소
            elif key == ord('-') or key == ord('_'):
                auto_exposure = False
                self.camera.MV_CC_SetEnumValue("ExposureAuto", 0)  # 수동
                current_exposure = max(current_exposure / 1.5, 1000)  # 최소 1ms
                self.camera.MV_CC_SetFloatValue("ExposureTime", current_exposure)
                print(f"노출 시간 감소: {current_exposure:.0f}us")
            
            # a: 자동 노출 토글
            elif key == ord('a'):
                auto_exposure = not auto_exposure
                if auto_exposure:
                    self.camera.MV_CC_SetEnumValue("ExposureAuto", 2)  # 자동
                    print("자동 노출: ON")
                else:
                    self.camera.MV_CC_SetEnumValue("ExposureAuto", 0)  # 수동
                    print("자동 노출: OFF (수동)")
            
            # q 또는 ESC: 종료
            elif key == ord('q') or key == 27:
                print("\n사용자가 취소했습니다.")
                cv2.destroyAllWindows()
                return False
        
        return True
    
    def calibrate(self):
        """카메라 캘리브레이션 수행"""
        print("\n" + "=" * 70)
        print("카메라 캘리브레이션 수행 중...")
        print("=" * 70)
        
        if len(self.objpoints) < 15:
            print("오류: 최소 15장의 이미지가 필요합니다.")
            return False
        
        print(f"사용 이미지: {len(self.objpoints)}장")
        print("계산 중... (수십 초 소요될 수 있습니다)")
        
        # 캘리브레이션 수행
        img_size = (640, 480)
        
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, 
            self.imgpoints, 
            img_size,
            None, 
            None
        )
        
        self.rms_error = ret
        
        print("\n" + "=" * 70)
        print("캘리브레이션 완료!")
        print("=" * 70)
        print(f"\nRMS 재투영 오차: {self.rms_error:.4f} pixels")
        print(f"  (0.5 미만: 우수, 1.0 미만: 양호, 1.0 이상: 재촬영 권장)")
        
        print(f"\n카메라 내부 파라미터 (Camera Matrix):")
        print(self.camera_matrix)
        
        print(f"\n왜곡 계수 (Distortion Coefficients):")
        print(self.dist_coeffs.ravel())
        
        return True
    
    def save_calibration(self, filename="hikrobot_calibration.npz"):
        """캘리브레이션 결과 저장"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename_with_time = f"hikrobot_calibration_{timestamp}.npz"
        
        np.savez(
            filename_with_time,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            rms_error=self.rms_error,
            chessboard_size=self.chessboard_size,
            square_size=self.square_size,
            num_images=len(self.objpoints)
        )
        
        print(f"\n캘리브레이션 결과 저장: {filename_with_time}")
        
        # 텍스트 형식으로도 저장
        txt_filename = filename_with_time.replace('.npz', '.txt')
        with open(txt_filename, 'w') as f:
            f.write("HIKRobot MV-CE050-30UC Camera Calibration Results\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Chessboard Size: {self.chessboard_size[0]}x{self.chessboard_size[1]}\n")
            f.write(f"Square Size: {self.square_size} mm\n")
            f.write(f"Number of Images: {len(self.objpoints)}\n")
            f.write(f"RMS Reprojection Error: {self.rms_error:.4f} pixels\n\n")
            f.write("Camera Matrix:\n")
            f.write(str(self.camera_matrix) + "\n\n")
            f.write("Distortion Coefficients:\n")
            f.write(str(self.dist_coeffs.ravel()) + "\n")
        
        print(f"텍스트 결과 저장: {txt_filename}")
        
        return filename_with_time


def main():
    print("\n" + "=" * 70)
    print("HIKRobot 카메라 캘리브레이션")
    print("=" * 70)
    
    # 체커보드 설정 (명령줄 인자로 변경 가능)
    if len(sys.argv) >= 4:
        try:
            chessboard_size = (int(sys.argv[1]), int(sys.argv[2]))
            square_size = float(sys.argv[3])
            print(f"\n체커보드 설정 (명령줄 인자):")
            print(f"  크기: {chessboard_size[0]}x{chessboard_size[1]}")
            print(f"  사각형 크기: {square_size}mm")
        except:
            print("\n잘못된 인자. 기본값 사용.")
            chessboard_size = (9, 6)
            square_size = 25.0
    else:
        # 기본값 사용
        chessboard_size = (9, 6)
        square_size = 25.0
        print("\n체커보드 설정 (기본값):")
        print(f"  크기: {chessboard_size[0]}x{chessboard_size[1]} (내부 코너)")
        print(f"  사각형 크기: {square_size}mm")
        print(f"\n다른 설정을 사용하려면:")
        print(f"  python camera_calibration_hikrobot.py [가로] [세로] [크기mm]")
        print(f"  예: python camera_calibration_hikrobot.py 9 6 25")
    
    # 캘리브레이션 객체 생성
    calib = HikRobotCalibration(chessboard_size, square_size)
    
    try:
        # 카메라 초기화
        if not calib.init_camera():
            print("\n카메라 초기화 실패!")
            return
        
        # 이미지 캡처
        if not calib.capture_images():
            print("\n캘리브레이션 취소.")
            return
        
        # 캘리브레이션 수행
        if not calib.calibrate():
            print("\n캘리브레이션 실패!")
            return
        
        # 결과 저장
        saved_file = calib.save_calibration()
        
        print("\n" + "=" * 70)
        print("모든 작업 완료!")
        print("=" * 70)
        print(f"\n다음 단계:")
        print(f"  1. 캘리브레이션 파일 사용: {saved_file}")
        print(f"  2. 로봇-카메라 통합 데이터 수집 시작")
        print(f"     python robot_camera_logger.py")
        print("=" * 70)
        
    except KeyboardInterrupt:
        print("\n\n사용자가 중단했습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        calib.close_camera()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
