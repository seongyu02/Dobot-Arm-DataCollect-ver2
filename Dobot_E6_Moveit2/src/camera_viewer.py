#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Viewer Module for HIKRobot Camera
"""

import sys
import os
import cv2
import numpy as np
from ctypes import *
import time
from typing import Optional, Tuple

# Windows 콘솔 UTF-8 인코딩 설정
if sys.platform == 'win32':
    try:
        import io
        if not isinstance(sys.stdout, io.TextIOWrapper) or (hasattr(sys.stdout, 'encoding') and sys.stdout.encoding.lower() != 'utf-8'):
            if hasattr(sys.stdout, 'buffer') and not sys.stdout.buffer.closed:
                sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
            if hasattr(sys.stderr, 'buffer') and not sys.stderr.buffer.closed:
                sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass

# Add parent directories to path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, parent_dir)

try:
    from MvImport.MvCameraControl_class import *
except ImportError:
    print("Error: HIKRobot SDK not found. Camera functionality will be disabled.")
    MvCamera = None


class HikRobotCamera:
    """HIKRobot Camera Controller"""
    
    def __init__(self, calibration_file: Optional[str] = None):
        """
        Initialize HIKRobot camera
        
        Args:
            calibration_file: Path to camera calibration file (.npz)
        """
        self.camera = None
        self.deviceList = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_file = calibration_file
        self.initialized = False
        
    def load_calibration(self):
        """Load camera calibration"""
        if self.calibration_file and os.path.exists(self.calibration_file):
            try:
                data = np.load(self.calibration_file)
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
                print(f"Calibration loaded: {self.calibration_file}")
                return True
            except Exception as e:
                print(f"Failed to load calibration: {e}")
                return False
        return False
    
    def init_camera(self) -> bool:
        """
        Initialize HIKRobot camera
        
        Returns:
            True if successful
        """
        if MvCamera is None:
            print("HIKRobot SDK not available")
            return False
            
        try:
            # SDK 초기화
            ret = MvCamera.MV_CC_Initialize()
            if ret != 0:
                print(f"SDK initialization failed (0x{ret:x})")
                return False
            
            # 카메라 검색
            self.deviceList = MV_CC_DEVICE_INFO_LIST()
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
            
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, self.deviceList)
            if ret != 0:
                print(f"Camera enumeration failed (0x{ret:x})")
                return False
            
            if self.deviceList.nDeviceNum == 0:
                print("No camera found")
                return False
            
            print(f"Camera found: {self.deviceList.nDeviceNum}")
            
            # 카메라 열기
            self.camera = MvCamera()
            stDeviceInfo = cast(self.deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
            
            ret = self.camera.MV_CC_CreateHandle(stDeviceInfo)
            if ret != 0:
                print(f"Camera handle creation failed (0x{ret:x})")
                return False
            
            ret = self.camera.MV_CC_OpenDevice()
            if ret != 0:
                print(f"Camera open failed (0x{ret:x})")
                return False
            
            # 카메라 설정
            self.camera.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            
            # 자동 노출/게인
            try:
                self.camera.MV_CC_SetEnumValue("ExposureAuto", 2)  # Continuous
                self.camera.MV_CC_SetEnumValue("GainAuto", 2)  # Continuous
            except:
                pass
            
            # 스트리밍 시작
            ret = self.camera.MV_CC_StartGrabbing()
            if ret != 0:
                print(f"Streaming start failed (0x{ret:x})")
                return False
            
            # 캘리브레이션 로드
            self.load_calibration()
            
            self.initialized = True
            print("Camera initialized successfully")
            return True
            
        except Exception as e:
            print(f"Camera initialization error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Get frame from camera
        
        Returns:
            (success, frame) tuple
        """
        if not self.initialized or self.camera is None:
            return False, None
            
        try:
            buffer_size = 2448 * 2048 * 3
            pData = (c_ubyte * buffer_size)()
            stFrameInfo = MV_FRAME_OUT_INFO_EX()
            memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
            
            ret = self.camera.MV_CC_GetOneFrameTimeout(pData, buffer_size, stFrameInfo, 1000)
            
            if ret == 0:
                image_data = np.frombuffer(pData, dtype=np.uint8, count=stFrameInfo.nFrameLen)
                
                # Bayer 형식 변환
                if stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerRG2BGR)
                elif stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerGB2BGR)  # BayerGR2BGR 대신 BayerGB2BGR 사용
                elif stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerGB2BGR)
                elif stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8:
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    image = cv2.cvtColor(image, cv2.COLOR_BayerBG2BGR)
                else:
                    if len(image_data) == stFrameInfo.nHeight * stFrameInfo.nWidth:
                        image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                    else:
                        image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, -1))
                
                # 640x480으로 리사이즈
                image = cv2.resize(image, (640, 480))
                
                # 캘리브레이션 보정 적용
                if self.camera_matrix is not None and self.dist_coeffs is not None:
                    image = cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
                
                # BGR → RGB 변환
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                
                return True, image
            else:
                return False, None
                
        except Exception as e:
            return False, None
    
    def cleanup(self):
        """Cleanup camera resources"""
        if self.camera:
            try:
                self.camera.MV_CC_StopGrabbing()
                self.camera.MV_CC_CloseDevice()
                self.camera.MV_CC_DestroyHandle()
            except:
                pass
        
        try:
            if MvCamera:
                MvCamera.MV_CC_Finalize()
        except:
            pass
        
        self.initialized = False
