import numpy as np
import time
from enum import Enum, auto
from dataclasses import dataclass
from typing import List, Tuple
import math
import random

class DroneState(Enum):
    IDLE = auto()
    SCANNING = auto()
    MAPPING = auto()
    PATH_PLANNING = auto()
    CLEANING = auto()
    RETURNING = auto()
    EMERGENCY = auto()

@dataclass
class Window:
    id: int
    corners: List[Tuple[float, float, float]]  # 3D coordinates
    center: Tuple[float, float, float]
    size: Tuple[float, float]  # width, height
    cleaned: bool = False

class CleaningDrone:
    def __init__(self):
        self.state = DroneState.IDLE
        self.position = (0.0, 0.0, 0.0)
        self.home_position = (0.0, 0.0, 0.0)
        self.battery = 100.0  # percentage
        self.cleaning_fluid = 100.0  # percentage
        self.windows: List[Window] = []
        self.cleaning_path: List[Tuple[float, float, float]] = []
        self.speed = 1.0  # m/s
        self.cleaning_speed = 0.5  # m²/min
        self.max_altitude = 50.0  # meters
        self.camera_fov = (60, 40)  # degrees

    def takeoff(self) -> bool:
        if self.state != DroneState.IDLE:
            print("Cannot takeoff - drone not idle")
            return False
        print("Taking off...")
        self.state = DroneState.SCANNING
        self.position = (0, 0, 5)  # Takeoff to 5m altitude
        time.sleep(2)
        return True

    def land(self) -> bool:
        print("Landing...")
        self.move_to_position(self.home_position)
        self.state = DroneState.IDLE
        return True

    def scan_building(self, scan_time: int = 60) -> bool:
        if self.state != DroneState.SCANNING:
            print("Cannot scan - drone not in scanning mode")
            return False
        print(f"Scanning building for {scan_time} seconds...")
        start_time = time.time()
        while time.time() - start_time < scan_time:
            if random.random() < 0.1:
                self._detect_window()
            self.battery -= 0.05
            if self.battery < 15:
                self._trigger_low_battery()
                break
            time.sleep(1)
        self.state = DroneState.MAPPING
        print("Scanning complete. Processing window data...")
        self._process_window_data()
        return True

    def _detect_window(self) -> None:
        window_id = len(self.windows) + 1
        width = random.uniform(0.8, 2.5)
        height = random.uniform(0.8, 1.8)
        x = self.position[0] + random.uniform(-5, 5)
        y = self.position[1] + random.uniform(2, 10)
        z = self.position[2] + random.uniform(-2, 2)
        corners = [
            (x, y, z),
            (x + width, y, z),
            (x + width, y + height, z),
            (x, y + height, z)
        ]
        center = (x + width / 2, y + height / 2, z)
        self.windows.append(Window(
            id=window_id,
            corners=corners,
            center=center,
            size=(width, height)
        ))
        print(f"Detected window {window_id} at position {center}")

    def _process_window_data(self) -> None:
        unique_windows = []
        seen_positions = set()
        for window in self.windows:
            pos_key = (round(window.center[0], 1),
                       round(window.center[1], 1),
                       round(window.center[2], 1))
            if pos_key not in seen_positions:
                seen_positions.add(pos_key)
                unique_windows.append(window)
        self.windows = unique_windows
        print(f"Identified {len(self.windows)} unique windows")

    def plan_cleaning_path(self, strategy: str = "zigzag") -> bool:
        if not self.windows:
            print("No windows detected - scan first")
            return False
        self.state = DroneState.PATH_PLANNING
        print("Generating cleaning path...")
        sorted_windows = sorted(self.windows, key=lambda w: w.center[1])
        for window in sorted_windows:
            approach_distance = 0.5
            approach_point = (
                window.center[0],
                window.center[1],
                window.center[2] - approach_distance
            )
            self.cleaning_path.append(approach_point)
            cleaning_points = self._generate_cleaning_pattern(
                window.corners, spacing=0.3
            )
            self.cleaning_path.extend(cleaning_points)
        if self.cleaning_path:
            self.cleaning_path.append(self.cleaning_path[0])
        print(f"Generated path with {len(self.cleaning_path)} waypoints")
        return True

    def _generate_cleaning_pattern(self, corners: List[Tuple], spacing: float) -> List[Tuple]:
        points = []
        min_x = min(c[0] for c in corners)
        max_x = max(c[0] for c in corners)
        min_y = min(c[1] for c in corners)
        max_y = max(c[1] for c in corners)
        z_pos = corners[0][2]
        current_y = min_y
        while current_y <= max_y:
            points.append((min_x, current_y, z_pos))
            points.append((max_x, current_y, z_pos))
            current_y += spacing
        return points

    def execute_cleaning(self) -> bool:
        if not self.cleaning_path:
            print("No cleaning path - plan path first")
            return False
        self.state = DroneState.CLEANING
        print("Starting cleaning sequence...")
        for i, point in enumerate(self.cleaning_path):
            if self.battery < 10:
                print("Critical battery level! Aborting cleaning.")
                self._trigger_emergency()
                return False
            if self.cleaning_fluid < 5:
                print("Out of cleaning fluid! Aborting cleaning.")
                self.state = DroneState.RETURNING
                break
            if not self._safe_move(point):
                print("Movement failed! Aborting cleaning.")
                self._trigger_emergency()
                return False
            if i % 10 > 1:
                self._activate_cleaning()
            self.battery -= 0.1
            self.cleaning_fluid -= 0.2
            for window in self.windows:
                dist = math.sqrt(
                    (window.center[0] - point[0])**2 +
                    (window.center[1] - point[1])**2 +
                    (window.center[2] - point[2])**2
                )
                if dist < 1.0:
                    window.cleaned = True
        print("Cleaning sequence complete")
        self.state = DroneState.RETURNING
        self.return_to_home()
        return True

    def _safe_move(self, target: Tuple) -> bool:
        distance = math.sqrt(
            (target[0] - self.position[0])**2 +
            (target[1] - self.position[1])**2 +
            (target[2] - self.position[2])**2
        )
        if distance > 10:
            print("Movement distance too large - breaking into segments")
            steps = int(distance // 5) + 1
            for i in range(steps):
                intermediate = (
                    self.position[0] + (target[0] - self.position[0]) * (i + 1) / steps,
                    self.position[1] + (target[1] - self.position[1]) * (i + 1) / steps,
                    self.position[2] + (target[2] - self.position[2]) * (i + 1) / steps
                )
                if not self._safe_move(intermediate):
                    return False
            return True
        if target[1] > self.max_altitude:
            print(f"Cannot exceed max altitude of {self.max_altitude}m")
            return False
        move_time = distance / self.speed
        print(f"Moving to {target} (distance: {distance:.2f}m, time: {move_time:.1f}s)")
        time.sleep(min(move_time, 0.1))
        self.position = target
        return True

    def _activate_cleaning(self) -> None:
        print("Activating cleaning system - spraying fluid and wiping")
        time.sleep(0.5)

    def _trigger_low_battery(self) -> None:
        print("Warning: Low battery!")
        self.state = DroneState.RETURNING
        self.return_to_home()

    def _trigger_emergency(self) -> None:
        print("EMERGENCY! Stopping all operations and landing immediately")
        self.state = DroneState.EMERGENCY
        self.land()

    def return_to_home(self) -> bool:
        if self.state == DroneState.EMERGENCY:
            return False
        print("Returning to home position...")
        self.state = DroneState.RETURNING
        self._safe_move(self.home_position)
        self.land()
        return True

    def recharge(self) -> None:
        print("Recharging battery...")
        time.sleep(2)
        self.battery = 100.0
        print("Battery fully charged")

    def refill_fluid(self) -> None:
        print("Refilling cleaning fluid...")
        time.sleep(1.5)
        self.cleaning_fluid = 100.0
        print("Cleaning fluid refilled")


# ✅ Example Usage
if __name__ == "__main__":
    drone = CleaningDrone()
    drone.takeoff()
    drone.scan_building(scan_time=10)  # Shortened scan time for testing
    drone.plan_cleaning_path()
    drone.execute_cleaning()
    drone.recharge()
    drone.refill_fluid()

    cleaned = sum(1 for w in drone.windows if w.cleaned)
    print(f"Mission complete. Cleaned {cleaned}/{len(drone.windows)} windows")
