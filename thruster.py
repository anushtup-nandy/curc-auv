import time
import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit

# Constants
NUM_THRUSTERS = 8
PWM_FREQ = 50  # PWM frequency in Hz
MIN_PULSE = 1100  # Minimum pulse width in microseconds
MAX_PULSE = 1900  # Maximum pulse width in microseconds
STOPPED_PULSE = 1500  # Pulse width for stopped thruster

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        return output

class ThrusterController:
    def __init__(self):
        # Initialize I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialize PCA9685 board
        self.pca = adafruit_pca9685.PCA9685(i2c)
        self.pca.frequency = PWM_FREQ
        
        # Initialize ServoKit
        self.kit = ServoKit(channels=16)
        
        # Set up thrusters
        self.thrusters = [self.kit.continuous_servo[i] for i in range(NUM_THRUSTERS)]
        
        # Initialize PID controllers for each thruster
        self.pid_controllers = [PIDController(kp=1.0, ki=0.1, kd=0.05) for _ in range(NUM_THRUSTERS)]
        
        # Stop all thrusters initially
        self.stop_all_thrusters()

    def set_thruster_speed(self, thruster_index, speed):
        """
        Set the speed of a specific thruster.
        :param thruster_index: Index of the thruster (0-7)
        :param speed: Speed value between -1 (full reverse) and 1 (full forward)
        """
        if thruster_index < 0 or thruster_index >= NUM_THRUSTERS:
            raise ValueError("Invalid thruster index")
        
        if speed < -1 or speed > 1:
            raise ValueError("Speed must be between -1 and 1")
        
        # Convert speed to pulse width
        pulse_width = STOPPED_PULSE + (speed * (MAX_PULSE - STOPPED_PULSE))
        
        # Set the pulse width for the thruster
        self.thrusters[thruster_index].set_pulse_width_range(MIN_PULSE, MAX_PULSE)
        self.thrusters[thruster_index].fraction = (pulse_width - MIN_PULSE) / (MAX_PULSE - MIN_PULSE)

    def set_thruster_speed_with_pid(self, thruster_index, setpoint, measured_value, dt):
        """
        Set the speed of a specific thruster using PID control.
        :param thruster_index: Index of the thruster (0-7)
        :param setpoint: Desired speed value between -1 and 1
        :param measured_value: Actual measured speed value
        :param dt: Time delta since last update
        """
        if thruster_index < 0 or thruster_index >= NUM_THRUSTERS:
            raise ValueError("Invalid thruster index")
        
        pid_output = self.pid_controllers[thruster_index].compute(setpoint, measured_value, dt)
        
        # Clamp PID output to valid speed range
        speed = max(min(pid_output, 1), -1)
        
        self.set_thruster_speed(thruster_index, speed)

    def stop_all_thrusters(self):
        """Stop all thrusters."""
        for thruster in self.thrusters:
            thruster.set_pulse_width_range(MIN_PULSE, MAX_PULSE)
            thruster.fraction = (STOPPED_PULSE - MIN_PULSE) / (MAX_PULSE - MIN_PULSE)

    def test_thrusters(self):
        """Test all thrusters by running them forward and reverse."""
        for i in range(NUM_THRUSTERS):
            print(f"Testing thruster {i}")
            self.set_thruster_speed(i, 0.2)  # 20% forward
            time.sleep(2)
            self.set_thruster_speed(i, -0.2)  # 20% reverse
            time.sleep(2)
            self.set_thruster_speed(i, 0)  # Stop
            time.sleep(1)

    def test_thrusters_with_pid(self):
        """Test all thrusters using PID control."""
        for i in range(NUM_THRUSTERS):
            print(f"Testing thruster {i} with PID")
            start_time = time.time()
            last_time = start_time
            
            for _ in range(100):  # Run for 100 iterations
                current_time = time.time()
                dt = current_time - last_time
                
                # Simulate a measured value (in a real scenario, this would come from a sensor)
                measured_value = 0  # Simulated measured speed
                
                # Set the desired speed (setpoint) to oscillate between -0.2 and 0.2
                setpoint = 0.2 * math.sin((current_time - start_time) * math.pi)
                
                self.set_thruster_speed_with_pid(i, setpoint, measured_value, dt)
                
                last_time = current_time
                time.sleep(0.01)  # Small delay to prevent too rapid updates
            
            self.set_thruster_speed(i, 0)  # Stop
            time.sleep(1)

if __name__ == "__main__":
    controller = ThrusterController()
    
    try:
        print("Starting thruster test sequence...")
        controller.test_thrusters()
        print("Standard thruster test complete.")
        
        print("Starting PID-controlled thruster test sequence...")
        controller.test_thrusters_with_pid()
        print("PID-controlled thruster test complete.")
    
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    
    finally:
        print("Stopping all thrusters...")
        controller.stop_all_thrusters()
        print("All thrusters stopped.")