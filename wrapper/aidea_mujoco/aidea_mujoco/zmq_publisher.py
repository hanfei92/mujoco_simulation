# wrapper/aidea_mujoco/aidea_mujoco/zmq_publisher.py（改造后）
import zmq

class ZmqPublisher:
    def __init__(self, endpoint="tcp://*:5555"):
        self._endpoint = endpoint
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.bind(endpoint)

    @property
    def endpoint(self):
        return self._endpoint

    def publish(self, topic: str, payload: bytes):
        from aidea_mujoco.msg import common_pb2
        env = common_pb2.ZmqEnvelope()
        env.topic = topic
        env.payload = payload
        self._sock.send(env.SerializeToString())

    def close(self):
        self._sock.close()
        self._ctx.term()