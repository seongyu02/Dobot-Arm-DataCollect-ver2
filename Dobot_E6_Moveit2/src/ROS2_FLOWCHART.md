# ROS2 Synchronized Recording — System Flowchart

## 전체 데이터 흐름

```mermaid
flowchart TD
    subgraph HW["Hardware"]
        HIK["HIK Robot Camera\n(Wrist, USB)"]
        ZED["ZED 2i Camera\n(Scene, USB3)"]
        DOBOT["Dobot E6\n(TCP/IP 192.168.5.1)"]
    end

    subgraph SERVER["robot_server.py  —  FastAPI (port 8000)"]
        direction TB

        subgraph CAM_LOOPS["Camera Grab Threads"]
            HIK_LOOP["_hik_grab_loop\n(daemon thread)"]
            ZED_LOOP["_zed_grab_loop\n(daemon thread)"]
        end

        subgraph BUFS["MJPEG Buffer (공유)"]
            BUF_HIK["_buf_hik_np / _buf_hik_jpg"]
            BUF_ZED["_buf_zed_np / _buf_zed_jpg"]
        end

        ROBOT_PUB["_robot_pub_loop\n~50 Hz (daemon thread)"]
        LEGACY["_record_loop / _record_tick\n(fallback: ZED 미연결 시)"]
        API["FastAPI REST + WebSocket\n/connect /status /pick-place/*\n/camera/* /ws/logs"]
    end

    subgraph ROS2["ros2_recorder.py  —  ROS2 Humble Node (dobot_recorder)"]
        direction TB

        subgraph PUBS["Publishers"]
            P_HIK["/dobot/hik/image_raw\nsensor_msgs/Image"]
            P_ZED["/dobot/zed/image_raw\nsensor_msgs/Image"]
            P_JS["/dobot/joint_states\nsensor_msgs/JointState"]
            P_TCP["/dobot/tcp_pose\nstd_msgs/Float32MultiArray"]
        end

        SYNC["ApproximateTimeSynchronizer\nslop = 35 ms\nHIK + ZED"]
        CACHE["Robot State Cache\n_latest_joints / _latest_tcp\n_latest_gripper / _latest_mode"]
        SAVE_Q["Save Queue\n(maxsize=60)"]
        SAVE_W["_save_worker\n(daemon thread)"]
    end

    subgraph DISK["External Drive  /media/billye6/새 볼륨/Dobot/2CAM"]
        direction LR
        EP["Episode N/\n├─ images/hik/frame_XXXXXX.jpg  224×224\n├─ images/zed/frame_XXXXXX.jpg  224×224\n├─ robot_data.csv\n├─ dataset.npy\n└─ metadata.txt"]
    end

    %% Hardware → Server
    HIK -->|"RGB frame"| HIK_LOOP
    ZED -->|"RGB frame"| ZED_LOOP
    DOBOT -->|"feedBackData() TCP"| ROBOT_PUB

    %% Grab loops → buffer + publish
    HIK_LOOP -->|"BGR + timestamp"| BUF_HIK
    HIK_LOOP -->|"publish_hik(bgr)"| P_HIK
    ZED_LOOP -->|"BGR + timestamp"| BUF_ZED
    ZED_LOOP -->|"publish_zed(bgr)"| P_ZED

    %% Robot pub loop → cache + publish
    ROBOT_PUB -->|"publish_robot()"| P_JS
    ROBOT_PUB -->|"publish_robot()"| P_TCP
    ROBOT_PUB -->|"update cache"| CACHE

    %% ROS2 sync path
    P_HIK -->|"subscribe"| SYNC
    P_ZED -->|"subscribe"| SYNC
    SYNC -->|"_on_sync callback\n(35 ms 이내 쌍 보장)"| SAVE_Q
    CACHE -->|"read at sync time"| SAVE_Q
    SAVE_Q -->|"async"| SAVE_W
    SAVE_W -->|"cv2.imwrite + append record"| EP

    %% Legacy fallback path
    BUF_HIK -.->|"fallback\n(ZED 미연결)"| LEGACY
    BUF_ZED -.->|"fallback"| LEGACY
    LEGACY -.->|"_stop_and_save()"| EP

    %% API control
    API -->|"start_recording()"| ROS2
    API -->|"stop_recording() → data"| ROS2
    API -->|"MJPEG stream"| BUF_HIK
    API -->|"MJPEG stream"| BUF_ZED
```

---

## 동기화 핵심 원리

```mermaid
sequenceDiagram
    participant HIK as HIK grab thread
    participant ZED as ZED grab thread
    participant SYNC as ApproximateTime<br/>Synchronizer
    participant CACHE as Robot State Cache
    participant ROBOT as _robot_pub_loop
    participant DISK as Disk

    HIK->>SYNC: Image + stamp_hik (캡처 직후)
    ZED->>SYNC: Image + stamp_zed (캡처 직후)
    ROBOT->>CACHE: joints/tcp/gripper @ 50Hz

    Note over SYNC: |stamp_hik - stamp_zed| ≤ 35ms 일 때만 콜백 발동

    SYNC->>DISK: hik_frame + zed_frame + robot_state(cache)
    Note over DISK: 세 데이터가 동일 시각 기준으로 저장됨
```

---

## 모드별 동작 요약

| 조건 | 레코딩 경로 | 동기화 품질 |
|---|---|---|
| ROS2 설치 + HIK + ZED | `ros2_recorder` sync callback | HIK↔ZED 35ms 이내 보장 |
| ROS2 설치 + HIK only | legacy `_record_tick` fallback | 버퍼 지연 최대 50ms |
| ROS2 미설치 | legacy `_record_tick` fallback | 버퍼 지연 최대 50ms |

---

## 데이터 정합(align) 확인 방법

### 1. 서버 로그 (WebSocket `/ws/logs`)
```
[09:34:11] Recording started → .../1 (ZED=ON, mode=ROS2+sync)
[09:34:45] [ros2_recorder] stopped — 680 synced frames
```
`mode=ROS2+sync` + synced frames 수로 동기화 정상 동작 확인.

### 2. `/status` 엔드포인트 (추후 추가 예정)
```json
{
  "ros2_sync_frames": 680,
  "ros2_sync_hz": 19.8,
  "ros2_mode": "sync"
}
```

### 3. 수집 후 오프라인 검증
```python
import numpy as np
data = np.load("dataset.npy", allow_pickle=True)
timestamps = [d['timestamp'] for d in data]
diffs = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
print(f"평균 간격: {sum(diffs)/len(diffs)*1000:.1f} ms")  # 목표: ~50ms (20Hz)
print(f"최대 간격: {max(diffs)*1000:.1f} ms")              # 이상치 확인
```
