#!/usr/bin/env python3
"""Onero A1 ACT 推理服务（官方 predict_action 管线）: TCP 8000。"""
import base64, json, socket, struct, time, logging
import numpy as np, cv2, torch
from lerobot.policies.act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.common.control_utils import predict_action

CKPT = "/home/am430/models/pretrained_model"
HOST, PORT = "0.0.0.0", 8000

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("infer")

def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf

def decode(b64):
    arr = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)  # BGR
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # RGB HWC uint8

def handle(conn, policy, preprocessor, postprocessor, device):
    while True:
        hdr = recv_exact(conn, 4)
        ln = struct.unpack(">I", hdr)[0]
        req = json.loads(recv_exact(conn, ln))
        t0 = time.time()
        state = np.asarray(req["state"], dtype=np.float32)
        top = decode(req["top"])
        wrist = decode(req["wrist"])
        observation = {
            "observation.state": state,
            "observation.images.top": top,
            "observation.images.wrist": wrist,
        }
        action = predict_action(
            observation, policy, device, preprocessor, postprocessor,
            use_amp=False, task="推", robot_type="onero_a1_single_arm",
        )
        act = [float(x) for x in action.reshape(-1)]
        resp = json.dumps({"action": act}).encode()
        conn.sendall(struct.pack(">I", len(resp)) + resp)
        log.info("infer %.1fms", (time.time() - t0) * 1000)

def main():
    log.info("loading policy from %s ...", CKPT)
    policy = ACTPolicy.from_pretrained(CKPT)
    policy.eval()
    device = torch.device("cuda")
    preprocessor, postprocessor = make_pre_post_processors(policy.config, pretrained_path=CKPT)
    log.info("policy + processors ready on %s", device)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(4)
    log.info("listening on %s:%d", HOST, PORT)
    while True:
        conn, addr = srv.accept()
        log.info("client connected: %s", addr)
        try:
            handle(conn, policy, preprocessor, postprocessor, device)
        except Exception as e:
            log.warning("conn ended: %r", e)
        finally:
            conn.close()

if __name__ == "__main__":
    main()
