class RobotPlugin:
    """机器人插件基类。子类实现 step / set_command / on_load。"""

    def __init__(self, mujoco_server, config: dict):
        self.mujoco_server = mujoco_server
        self.config = config
        self.robot_id = config["robot_id"]
        self.step_counter = 0

    def on_load(self, mjmodel, mjdata):
        pass

    def step(self, mjdata):
        pass

    def set_command(self, cmd: dict):
        pass

    def on_close(self):
        pass
