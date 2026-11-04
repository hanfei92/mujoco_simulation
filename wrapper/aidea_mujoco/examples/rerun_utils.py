import queue
import threading
import numpy as np
import rerun as rr
import cv2


class RerunLogger:
    def __init__(self, maxsize: int = 2):
        self._log_queue = queue.Queue(maxsize=maxsize)
        self._log_thread = None
        self._stop_event = threading.Event()

    def _logging_worker(self):
        while not self._stop_event.is_set():
            try:
                item = self._log_queue.get(timeout=0.1)
                kind, payload, label = item

                if kind == "points":
                    points = payload
                    if isinstance(points, list):
                        for points_name, points_instance in points:
                            rr.log(f"{label}/{points_name}", rr.Points3D(points_instance))
                    else:
                        rr.log(label, rr.Points3D(points))

                elif kind == "image":
                    image, is_depth = payload
                    if is_depth:
                        # uint16 灰度深度
                        rr.log(label, rr.Image(image, color_model="L"))
                    else:
                        if image.ndim == 3:
                            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                        rr.log(label, rr.Image(image))

                self._log_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                # 别吞异常，至少打出来
                print(f"[RerunLogger] log failed: {e}")

    def init_pointcloud_viz(self):
        rr.init("Aidea Mujoco", spawn=False)
        rr.spawn(memory_limit='5GB')
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log(
            "world/xyz",
            rr.Arrows3D(
                vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            ),
        )
        self._stop_event.clear()
        self._log_thread = threading.Thread(target=self._logging_worker, daemon=True)
        self._log_thread.start()

    def update_pointcloud_viz(self, points: np.ndarray | list[tuple[str, np.ndarray]], label: str):
        if self._log_thread is None or not self._log_thread.is_alive():
            raise RuntimeError(
                "RerunLogger must be initialized via init_pointcloud_viz() before calling update_pointcloud_viz()."
            )
        if self._stop_event.is_set():
            return
        self._put(("points", points, label))

    # def update_camera_viz(self, camera_name: str, image: np.ndarray, is_depth: bool = False):
    #     """把相机图 log 到 Rerun。"""
    #     if self._log_thread is None or not self._log_thread.is_alive():
    #         raise RuntimeError(
    #             "RerunLogger must be initialized via init_pointcloud_viz() first."
    #         )
    #     if self._stop_event.is_set():
    #         return
        
    #     self._put(("image", (image, is_depth), f"camera/{camera_name}"))
    def update_camera_viz(self, camera_name, image, is_depth=False):
        if self._log_thread is None or not self._log_thread.is_alive():
            raise RuntimeError("init_pointcloud_viz() first.")
        if self._stop_event.is_set():
            return

        if is_depth:
            # float32 米 → uint16 毫米
            if image.dtype == np.float32 or image.dtype == np.float64:
                image = np.clip(image * 1000.0, 0, 65535).astype(np.uint16)
            elif image.dtype != np.uint16:
                image = image.astype(np.uint16)

        self._put(("image", (image, is_depth), f"camera/{camera_name}"))

    def _put(self, item):
        """非阻塞入队，满了就丢最旧的。"""
        while not self._stop_event.is_set():
            try:
                self._log_queue.put_nowait(item)
                break
            except queue.Full:
                try:
                    self._log_queue.get_nowait()
                except queue.Empty:
                    pass

    def stop(self):
        self._stop_event.set()
        if self._log_thread:
            self._log_thread.join(timeout=2.0)