import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class VirtualIMU:
    """Simulates IMU sensor data for yaw orientation"""
    def __init__(self):
        self.yaw = 0.0  # Current orientation in radians
    
    def update(self, new_yaw):
        self.yaw = new_yaw % (2 * np.pi)
    
    def read(self):
        return self.yaw

class OmniRobotController:
    """Controls omnidirectional robot with position and orientation control"""
    def __init__(self, start_pos, imu, pid_orient=(2.0, 0.1, 0.5), pid_pos=(0.8, 0.0, 0.2)):
        self.position = np.array(start_pos, dtype=np.float32)
        self.imu = imu
        self.trajectory = [self.position.copy()]
        
        # PID parameters
        self.Kp_o, self.Ki_o, self.Kd_o = pid_orient  # Orientation control
        self.Kp_p, self.Ki_p, self.Kd_p = pid_pos     # Position control
        
        # Control states
        self.integral_orient = 0.0
        self.prev_error_orient = 0.0
        self.integral_pos = np.zeros(2)
        self.prev_error_pos = np.zeros(2)
        
        # Target positions (quadrant sequence)
        self.targets = [
            np.array([5.0, 5.0]),    # x,y
            np.array([-5.0, 5.0]),   # -x,y
            np.array([-5.0, -5.0]),  # -x,-y
            np.array([5.0, -5.0])    # x,-y
        ]
        self.current_target = 0

    def calculate_desired_yaw(self):
        """Calculate yaw needed to face origin"""
        dx = -self.position[0]
        dy = -self.position[1]
        return np.arctan2(dy, dx)

    def update_orientation(self, dt):
        """PID control for orientation"""
        current_yaw = self.imu.read()
        target_yaw = self.calculate_desired_yaw()
        
        # Wrapped angular error
        error = (target_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi
        
        # PID calculations
        self.integral_orient += error * dt
        derivative = (error - self.prev_error_orient) / dt
        control = self.Kp_o * error + self.Ki_o * self.integral_orient + self.Kd_o * derivative
        self.prev_error_orient = error
        
        # Update orientation
        self.imu.update(current_yaw + control * dt)

    def update_position(self, dt):
        """PID control for position"""
        target = self.targets[self.current_target]
        error = target - self.position
        
        # PID calculations
        self.integral_pos += error * dt
        derivative = (error - self.prev_error_pos) / dt
        control = self.Kp_p * error + self.Ki_p * self.integral_pos + self.Kd_p * derivative
        
        # Update position
        self.position += control * dt
        self.trajectory.append(self.position.copy())
        self.prev_error_pos = error
        
        # Check target proximity
        if np.linalg.norm(error) < 0.3:  # Target radius threshold
            self.current_target = (self.current_target + 1) % 4
            self.integral_pos = np.zeros(2)
            self.prev_error_pos = np.zeros(2)

def create_animation(controller, imu):
    """Create and display the animation"""
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Initialize plot elements
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_title("Quadrant Navigation with Origin Orientation")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.grid(True)
    
    # Create plot elements
    robot_dot, = ax.plot([], [], 'bo', markersize=10, label='Robot')
    path_line, = ax.plot([], [], 'b--', alpha=0.5, label='Path')
    orientation_arrow = ax.quiver([], [], [], [], color='g', scale=20, label='Orientation')
    target_marker = ax.plot([], [], 'rx', markersize=15, label='Target')[0]
    origin = ax.scatter(0, 0, c='r', marker='*', s=200, label='Origin')
    ax.legend()

    def init():
        robot_dot.set_data([], [])
        path_line.set_data([], [])
        orientation_arrow.set_offsets(np.empty((0, 2)))
        orientation_arrow.set_UVC([], [])
        target_marker.set_data([], [])
        return robot_dot, path_line, orientation_arrow, target_marker

    def update(frame):
        # Update controller state
        controller.update_orientation(0.1)
        controller.update_position(0.1)
        
        # Update plot elements
        x, y = controller.position
        path = np.array(controller.trajectory)
        current_target = controller.targets[controller.current_target]
        
        # Update positions
        robot_dot.set_data(x, y)
        path_line.set_data(path[:,0], path[:,1])
        target_marker.set_data(current_target[0], current_target[1])
        
        # Update orientation arrow
        yaw = imu.read()
        orientation_arrow.set_offsets([[x, y]])
        orientation_arrow.set_UVC(np.cos(yaw), np.sin(yaw))
        
        # Adjust view if needed
        ax.set_xlim(min(-10, x-2), max(10, x+2))
        ax.set_ylim(min(-10, y-2), max(10, y+2))
        
        return robot_dot, path_line, orientation_arrow, target_marker

    ani = FuncAnimation(
        fig, 
        update, 
        frames=200,
        init_func=init, 
        blit=True,
        interval=100,
        repeat=False
    )
    
    plt.show()
    
    # Final trajectory plot
    fig2, ax2 = plt.subplots(figsize=(8, 8))
    path = np.array(controller.trajectory)
    ax2.plot(path[:,0], path[:,1], 'b-', label='Robot Path')
    ax2.scatter(0, 0, c='r', marker='*', s=200, label='Origin')
    
    # Plot target points
    targets = np.array(controller.targets)
    ax2.scatter(targets[:,0], targets[:,1], c='black', marker='x', s=100, label='Targets')
    
    # Add orientation arrows
    step = max(1, len(path)//20)
    ax2.quiver(path[::step,0], path[::step,1],
              np.cos([imu.read()]*len(path[::step])),
              np.sin([imu.read()]*len(path[::step])),
              scale=20, color='g', alpha=0.7)
    
    ax2.set_title("Final Quadrant Navigation Path")
    ax2.legend()
    ax2.grid(True)
    plt.show()

if __name__ == "__main__":
    # Initialize system with PID controllers
    imu = VirtualIMU()
    controller = OmniRobotController(
        start_pos=[0.0, 0.0],  # Start at origin
        imu=imu,
        pid_orient=(2.0, 0.1, 0.5),  # Orientation PID
        pid_pos=(0.8, 0.0, 0.2)      # Position PID
    )
    
    # Create and run animation
    create_animation(controller, imu)