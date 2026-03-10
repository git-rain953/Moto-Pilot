class ComponentDetector:
    def __init__(self):
        print("[System] Component Detector initialized (Liu Xiaoyu Module)")

    def detect_handlebar(self, vehicle_roi):
        """
        在车辆区域内检测把手位置
        """
        print("[Perception] Detecting handlebar...")
        # TODO: Deep Learning / Point Cloud segmentation
        handle_pose = {"x": 0.5, "y": 0.2, "z": 1.1} # Relative to vehicle
        return handle_pose

    def detect_buttons(self, vehicle_roi):
        """
        检测启动按钮、喇叭等
        """
        print("[Perception] Detecting control buttons...")
        buttons = {
            "start": {"x": 0.45, "y": 0.1},
            "horn": {"x": 0.40, "y": 0.15}
        }
        return buttons
