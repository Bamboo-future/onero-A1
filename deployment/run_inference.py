#!/usr/bin/env python3
"""Onero A1 部署循环：读从臂+双摄 -> 发送到推理电脑 -> 执行返回动作。"""
import base64, json, socket, struct, time, logging
import cv2
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.configs import Cv2Backends, ColorMode
from lerobot.robots.onero_a1.onero_a1 import OneroA1SingleArm
from lerobot.robots.onero_a1.config_onero_a1 import OneroA1SingleArmConfig

SERVER = ("192.168.1.227", 8000)
# 训练数据中从臂起始姿态（中位数），部署前先平移到该姿态再推理
START_POSE = [-0.065, 0.103, 0.01, -0.019, -0.097, 0.061, 0.525]  # 后组 ep22-51 起始姿态
HZ = 10.0          # movej 平滑执行，10Hz 更新足够
MAX_DELTA = 0.1     # 每步最大关节变化 (rad)，安全限幅

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("deploy")

def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf

def jpeg_b64(rgb):
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return base64.b64encode(buf.tobytes()).decode()

def main():
    cfg = OneroA1SingleArmConfig(
        port="/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209636A54343-if00",
        robot_model="a1_l",
        control_mode="movej",  # 部署用 movej 平滑执行，避免 MIT 线程抢锁/抖动
        enable_safety=True,
        max_delta_rad=0.2,
        cameras={
            "top": OpenCVCameraConfig(
                index_or_path="/dev/v4l/by-path/pci-0000:04:00.4-usb-0:1:1.0-video-index0",
                width=640, height=480, fps=15, fourcc="MJPG",
                backend=Cv2Backends.V4L2, color_mode=ColorMode.RGB),
            "wrist": OpenCVCameraConfig(
                index_or_path="/dev/v4l/by-path/pci-0000:01:00.0-usb-0:2:1.0-video-index0",
                width=640, height=480, fps=15, fourcc="MJPG",
                backend=Cv2Backends.V4L2, color_mode=ColorMode.RGB),
        },
    )
    robot = OneroA1SingleArm(cfg)
    robot.connect(calibrate=True)
    log.info("从臂已连接，正在平移到训练起始姿态...")
    robot._arm.movej(START_POSE, speed_scale=0.3, trajectory_connect=0)
    log.info("已到达起始姿态，开始 movej 平滑推理控制；Ctrl+C 退出")

    conn = socket.create_connection(SERVER, timeout=5)
    loop_t = 1.0 / HZ
    try:
        while True:
            t0 = time.monotonic()
            obs = robot.get_observation()
            state = [float(obs[f"joint{i}.pos"]) for i in range(1, 8)]
            payload = json.dumps({
                "state": state,
                "top": jpeg_b64(obs["top"]),
                "wrist": jpeg_b64(obs["wrist"]),
            }).encode()
            conn.sendall(struct.pack(">I", len(payload)) + payload)
            resp = json.loads(recv_exact(conn, struct.unpack(">I", recv_exact(conn, 4))[0]))
            target = [state[i] + max(-MAX_DELTA, min(MAX_DELTA, resp["action"][i] - state[i])) for i in range(7)]
            robot._arm.movej(target, speed_scale=0.5, trajectory_connect=0)  # connect=0 立即执行（1 只缓冲不执行）
            elapsed = time.monotonic() - t0
            if elapsed > loop_t * 1.5:
                log.warning("loop slow: %.0fms", elapsed * 1000)
            time.sleep(max(loop_t - elapsed, 0))
    except KeyboardInterrupt:
        log.info("退出...")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        robot.disconnect()
        log.info("已断开并关闭电机")

if __name__ == "__main__":
    main()
