import time
from src.navigation.navigator import Navigator
from src.perception.detector import ComponentDetector
from src.interaction.boarder import BoardingManager
from src.control.rider import StabilityController, ManipulationController

class G1MotoSystem:
    def __init__(self):
        print("=== Initializing G1 Motorcycle System ===")
        self.navigator = Navigator()      # Li Haohao
        self.detector = ComponentDetector() # Liu Xiaoyu
        self.boarder = BoardingManager()  # Chen Zhitao
        self.stability = StabilityController() # Xuan Shihan
        self.manipulation = ManipulationController() # Yuan Tianbo
        
        self.state = "IDLE"

    def run(self):
        while True:
            if self.state == "IDLE":
                print("\n[State: IDLE] Waiting for command...")
                time.sleep(1)
                self.state = "SEARCHING" # Simulate user command

            elif self.state == "SEARCHING":
                print("\n[State: SEARCHING] Looking for motorcycle (Li Haohao)...")
                vehicle_pose = self.navigator.find_motorcycle()
                if vehicle_pose:
                    self.navigator.move_to_target(vehicle_pose)
                    self.state = "BOARDING_PREP"
                
            elif self.state == "BOARDING_PREP":
                print("\n[State: BOARDING_PREP] Analyzing vehicle parts (Liu Xiaoyu)...")
                vehicle_roi = {"x": 10, "y": 5, "w": 2, "h": 1}
                handles = self.detector.detect_handlebar(vehicle_roi)
                buttons = self.detector.detect_buttons(vehicle_roi)
                print(f"  Detected handles at: {handles}")
                self.state = "BOARDING"

            elif self.state == "BOARDING":
                print("\n[State: BOARDING] Mounting the bike (Chen Zhitao)...")
                approach_pose = self.boarder.calculate_approach_pose({"x": 10.5, "y": 5.2})
                success = self.boarder.execute_boarding()
                if success:
                    self.state = "RIDING"

            elif self.state == "RIDING":
                print("\n[State: RIDING] System Active (Xuan Shihan & Yuan Tianbo)...")
                # Simulate a control loop
                for i in range(3):
                    self.stability.balance_control({"pitch": 0.1, "roll": 0.02})
                    self.manipulation.twist_throttle(10 + i*5)
                    time.sleep(0.5)
                
                print("Task Completed. Stopping.")
                self.manipulation.brake(50)
                break

if __name__ == "__main__":
    system = G1MotoSystem()
    system.run()
