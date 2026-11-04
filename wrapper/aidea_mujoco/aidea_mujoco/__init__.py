import os
import sys

# Enable GPU acceleration by default on Linux
if sys.platform.startswith("linux"):
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    os.environ.setdefault("XLA_FLAGS", "--xla_gpu_triton_gemm_any=true")

from .mujoco_simulator import MujocoSimulator
from .aidea_mujoco_simulator import AideaMujocoSimulator
from .utils import models_path
