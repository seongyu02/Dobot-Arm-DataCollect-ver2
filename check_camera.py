"""
USB 카메라 연결 확인 프로그램
연결된 모든 카메라를 스캔하고 테스트합니다.

사용 방법:
python check_camera.py

💡 USB3 Vision 카메라를 찾을 수 없다면:
1. 장치 관리자 확인 (Win + X → 장치 관리자)
2. "카메라" 또는 "이미징 장치" 항목에서 카메라 확인
3. USB 3.0 포트에 연결되어 있는지 확인
4. 제조사의 전용 뷰어 소프트웨어로 먼저 테스트
"""

import cv2
import sys
import subprocess

def scan_cameras(max_index=20, backend=None):
    """연결된 카메라 스캔 (더 넓은 범위)"""
    print("\n" + "=" * 70)
    print("📷 USB 카메라 스캔 중 (인덱스 0~19)...")
    if backend:
        print(f"   백엔드: {backend}")
    print("=" * 70)
    
    available_cameras = []
    
    for i in range(max_index):
        try:
            # 백엔드 지정 (선택적)
            if backend == 'dshow':
                cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # DirectShow (Windows)
            elif backend == 'msmf':
                cap = cv2.VideoCapture(i, cv2.CAP_MSMF)   # Media Foundation
            else:
                cap = cv2.VideoCapture(i)
            
            if cap.isOpened():
                # 카메라 정보 가져오기
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = int(cap.get(cv2.CAP_PROP_FPS))
                backend_name = cap.getBackendName()
                
                # 실제로 프레임을 읽을 수 있는지 확인
                ret, frame = cap.read()
                if ret:
                    available_cameras.append({
                        'index': i,
                        'width': width,
                        'height': height,
                        'fps': fps,
                        'backend': backend_name
                    })
                    print(f"✅ 카메라 {i}: {width}x{height} @ {fps}fps [{backend_name}]")
                
                cap.release()
        except Exception as e:
            # 에러는 무시하고 계속 스캔
            pass
    
    return available_cameras

def test_camera(camera_index):
    """특정 카메라 테스트"""
    print(f"\n📹 카메라 {camera_index} 테스트 중...")
    print("   'q' 또는 'ESC' 키를 누르면 종료됩니다.")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"❌ 카메라 {camera_index}를 열 수 없습니다.")
        return False
    
    # 해상도 설정
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"✅ 해상도: {actual_width}x{actual_height}")
    
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("⚠️  프레임을 읽을 수 없습니다.")
                break
            
            frame_count += 1
            
            # 정보 표시
            display_frame = frame.copy()
            info_text = f"Camera {camera_index} | Frame {frame_count} | {actual_width}x{actual_height}"
            cv2.putText(display_frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.putText(display_frame, "Press 'q' or 'ESC' to quit", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow(f'Camera {camera_index} Test', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
        
        print(f"✅ 총 {frame_count}개 프레임 수신")
        return True
    
    except Exception as e:
        print(f"❌ 오류: {e}")
        return False
    
    finally:
        cap.release()
        cv2.destroyAllWindows()

def open_device_manager():
    """Windows 장치 관리자 열기"""
    try:
        subprocess.run(['devmgmt.msc'], shell=True)
        print("✅ 장치 관리자를 열었습니다.")
    except Exception as e:
        print(f"⚠️ 장치 관리자를 열 수 없습니다: {e}")

def main():
    print("\n" + "=" * 70)
    print("📷 USB 카메라 연결 확인 프로그램 (개선된 스캔)")
    print("=" * 70)
    
    # 장치 관리자 열기 옵션
    print("\n먼저 장치 관리자에서 카메라를 확인하시겠습니까?")
    open_dm = input("(y/n, Enter=n): ").strip().lower()
    if open_dm == 'y':
        open_device_manager()
        print("\n장치 관리자에서 다음 항목을 확인하세요:")
        print("  - '카메라' 또는 '이미징 장치'")
        print("  - USB3 Vision 카메라가 인식되어 있는지 확인")
        print("  - 노란색 느낌표(!)가 있으면 드라이버 문제\n")
        input("확인 후 Enter 키를 누르세요...")
    
    # 백엔드 선택
    print("\n" + "=" * 70)
    print("카메라 백엔드 선택:")
    print("  [1] 자동 (기본)")
    print("  [2] DirectShow (권장 - Windows)")
    print("  [3] Media Foundation")
    print("=" * 70)
    
    backend_choice = input("\n선택 (Enter=1): ").strip()
    
    if backend_choice == '2':
        backend = 'dshow'
        print("✅ DirectShow 백엔드 사용")
    elif backend_choice == '3':
        backend = 'msmf'
        print("✅ Media Foundation 백엔드 사용")
    else:
        backend = None
        print("✅ 자동 백엔드 사용")
    
    # 1. 카메라 스캔
    cameras = scan_cameras(max_index=20, backend=backend)
    
    if not cameras:
        print("\n" + "=" * 70)
        print("❌ 연결된 카메라를 찾을 수 없습니다.")
        print("=" * 70)
        
        print("\n📋 체크리스트:")
        print("  □ USB 케이블이 제대로 연결되어 있나요?")
        print("  □ 카메라 전원이 켜져 있나요? (USB3 Vision은 외부 전원 필요)")
        print("  □ USB 3.0 포트(파란색)에 연결되어 있나요?")
        print("  □ 다른 프로그램(Zoom, Teams 등)에서 카메라를 사용 중인가요?")
        print("  □ Windows 장치 관리자에서 카메라가 인식되나요?")
        
        print("\n💡 USB3 Vision 카메라의 경우:")
        print("  1. 제조사 전용 SDK/드라이버 설치 필요")
        print("  2. GenICam 호환 필요 (OpenCV로 직접 접근 어려울 수 있음)")
        print("  3. 제조사 뷰어 소프트웨어로 먼저 테스트 권장")
        
        print("\n🔧 해결 방법:")
        print("  1. 다른 USB 포트 시도 (특히 USB 3.0/3.1)")
        print("  2. 다른 백엔드 시도 (DirectShow, Media Foundation)")
        print("  3. 컴퓨터 재부팅")
        print("  4. 제조사 소프트웨어 설치 및 테스트")
        
        print("\n다시 시도하시겠습니까?")
        retry = input("(y/n): ").strip().lower()
        if retry == 'y':
            main()
        return
    
    print("\n" + "=" * 70)
    print(f"✅ 총 {len(cameras)}개의 카메라가 발견되었습니다.")
    print("=" * 70)
    
    # 2. 테스트할 카메라 선택
    if len(cameras) == 1:
        selected_camera = cameras[0]['index']
        print(f"\n자동 선택: 카메라 {selected_camera}")
        print(f"   해상도: {cameras[0]['width']}x{cameras[0]['height']}")
        print(f"   백엔드: {cameras[0]['backend']}")
    else:
        print("\n사용 가능한 카메라:")
        for cam in cameras:
            print(f"  [{cam['index']}] {cam['width']}x{cam['height']} @ {cam['fps']}fps [{cam['backend']}]")
        
        while True:
            try:
                choice = input(f"\n테스트할 카메라 번호: ").strip()
                selected_camera = int(choice)
                
                if any(cam['index'] == selected_camera for cam in cameras):
                    break
                else:
                    print("⚠️  유효한 카메라 번호를 입력하세요.")
            except ValueError:
                print("⚠️  숫자를 입력하세요.")
    
    # 3. 카메라 테스트
    print("\n" + "=" * 70)
    if test_camera(selected_camera):
        print("\n✅ 카메라 테스트 완료!")
        print("\n이제 camera_calibration.py를 실행하여 캘리브레이션을 진행하세요:")
        print(f"   python camera_calibration.py")
        print(f"   (카메라 인덱스: {selected_camera})")
    else:
        print("\n❌ 카메라 테스트 실패")
    print("=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n사용자가 중단했습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()
