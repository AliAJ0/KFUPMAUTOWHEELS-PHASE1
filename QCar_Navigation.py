# region: File Description and Imports
"""
Autonomous Taxi Service for QCar 2 Competition
Implements passenger pickup/dropoff and navigation
"""
import os
import time
import numpy as np
from threading import Thread
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF

# ===== Configuration Parameters =====
tf = 600  # Experiment duration (seconds)
controllerUpdateRate = 100  # Hz
v_ref = 0.8  # Base speed (m/s)
pickup_location = [0.125, 4.395]  # x,y (meters)
dropoff_location = [-0.905, 0.800]  # x,y (meters)
taxi_hub = [-1.205, -0.83]  # x,y (meters)

# PID Controller Gains
K_p = 0.2
K_i = 0.05
K_d = 0.01

# Stanley Controller Gain
K_stanley = 1.2

# LED Colors
LED_PICKUP = 1  # Green
LED_DROPOFF = 0  # Red
LED_NAVIGATING = 4  # Cyan

class QCarTaxiService:
    def __init__(self):
        self.KILL_THREAD = False
        self.current_mission = "to_pickup"
        self.stop_duration = 0
        self.stop_time = 0
        
        # Initialize QCar interface
        self.qcar = QCar(readMode=1, frequency=controllerUpdateRate)
        self.ekf = QCarEKF(x_0=taxi_hub + [0])  # Initial pose
        self.gps = QCarGPS(initialPose=taxi_hub + [0])
        
        # Mission waypoints
        self.waypoints = {
            'to_pickup': pickup_location,
            'to_dropoff': dropoff_location,
            'to_hub': taxi_hub
        }

    def run(self):
        with self.qcar, self.gps:
            t0 = time.time()
            while (time.time() - t0 < tf) and (not self.KILL_THREAD):
                # Get current position
                if self.gps.readGPS():
                    y_gps = np.array([self.gps.position[0], 
                                     self.gps.position[1], 
                                     self.gps.orientation[2]])
                    self.ekf.update([self.qcar.motorTach, self.delta], 
                                   dt, y_gps, self.qcar.gyroscope[2])
                
                x, y = self.ekf.x_hat[0,0], self.ekf.x_hat[1,0]
                
                # Mission state machine
                if self.current_mission == "to_pickup":
                    self.navigate_to(x, y, pickup_location, LED_NAVIGATING)
                    if self.check_arrival(x, y, pickup_location):
                        self.begin_stop(3, LED_PICKUP)
                        self.current_mission = "to_dropoff"
                
                elif self.current_mission == "to_dropoff":
                    self.navigate_to(x, y, dropoff_location, LED_NAVIGATING)
                    if self.check_arrival(x, y, dropoff_location):
                        self.begin_stop(3, LED_DROPOFF)
                        self.current_mission = "to_hub"
                
                elif self.current_mission == "to_hub":
                    self.navigate_to(x, y, taxi_hub, LED_NAVIGATING)
                    if self.check_arrival(x, y, taxi_hub):
                        self.current_mission = "to_pickup"
                
                # Handle stopping
                if self.stop_duration > 0:
                    if time.time() - self.stop_time >= self.stop_duration:
                        self.stop_duration = 0
                    else:
                        self.qcar.write(0, 0, self.stop_leds)
                        continue
                
                # Normal navigation
                self.execute_control(x, y)
    
    def navigate_to(self, x, y, target, led_color):
        distance = np.sqrt((target[0]-x)**2 + (target[1]-y)**2)
        if distance < 0.5:  # Slow down when approaching target
            self.v_ref = 0.3
        else:
            self.v_ref = 0.8
        self.set_leds(led_color)
    
    def begin_stop(self, duration, led_color):
        self.stop_duration = duration
        self.stop_time = time.time()
        self.stop_leds = led_color
    
    def check_arrival(self, x, y, target, threshold=0.2):
        return np.sqrt((target[0]-x)**2 + (target[1]-y)**2) < threshold
    
    def execute_control(self, x, y):
        # Implement Stanley controller or other navigation logic
        # ... [controller implementation] ...
        
        # Write to QCar
        self.qcar.write(self.u, self.delta, self.leds)
    
    def set_leds(self, color_id):
        if IS_PHYSICAL_QCAR:
            os.system(f'ros2 param set qcar2_hardware led_color_id {color_id}')
        self.leds = color_id

if __name__ == '__main__':
    taxi = QCarTaxiService()
    try:
        taxi.run()
    except KeyboardInterrupt:
        taxi.KILL_THREAD = True
