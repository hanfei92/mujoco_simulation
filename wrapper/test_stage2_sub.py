# wrapper/test_stage2_sub.py
"""Stage 2 SUB: 在宿主机验证 Wrapper 的 ZMQ PUB 数据通道。"""
import numpy as np
import zmq

from aidea_mujoco.msg import common_pb2, lidar_pb2


def main(endpoint="tcp://127.0.0.1:5555"):
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.setsockopt(zmq.RCVTIMEO, 10000)
    print(f"[sub] connected to {endpoint}, waiting for messages...")

    while True:
        try:
            raw = sub.recv()
        except zmq.Again:
            print("[sub] timeout: no message in 10s")
            break

        print(f"[sub] got {len(raw)} bytes")
        env = common_pb2.ZmqEnvelope()
        env.ParseFromString(raw)
        print(f"[sub] topic={env.topic}, payload_len={len(env.payload)}")

        if env.topic == "lidar":
            msg = lidar_pb2.PointCloud2()
            msg.ParseFromString(env.payload)
            print(f"[sub] lidar: width={msg.width}, data_len={len(msg.data)}, point_step={msg.point_step}")
            if msg.width == 0 or len(msg.data) == 0:
                print("[sub] empty lidar payload, continue")
                continue
            pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
            print(
                f"[sub] lidar: shape={pts.shape}, frame={msg.header.frame_id}, "
                f"x=[{pts[:,0].min():.2f}, {pts[:,0].max():.2f}], "
                f"y=[{pts[:,1].min():.2f}, {pts[:,1].max():.2f}], "
                f"z=[{pts[:,2].min():.2f}, {pts[:,2].max():.2f}]"
            )
            break

        elif env.topic == "image":
            msg = lidar_pb2.Image()
            msg.ParseFromString(env.payload)
            print(f"[sub] image: {msg.height}x{msg.width}, encoding={msg.encoding}")

    sub.close()
    ctx.term()


if __name__ == "__main__":
    main()