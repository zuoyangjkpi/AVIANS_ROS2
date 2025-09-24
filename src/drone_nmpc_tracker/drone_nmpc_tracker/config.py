#!/usr/bin/env python3
"""
NMPC Configuration for Drone Person Tracking
Based on formation_config.py from AirshipMPC
Adapted for ROS2 Jazzy and Python 3.12
"""

import math
import numpy as np

class NMPCConfig:
    """Configuration class for NMPC drone person tracking"""
    
    def __init__(self):
        # ========== NMPC Parameters ==========
        self.TIMESTEP = 0.25  # Time step in seconds
        self.LOOKAHEAD = 2.0  # Look ahead time in seconds (8 * TIMESTEP)
        self.N = int(self.LOOKAHEAD / self.TIMESTEP)  # Number of prediction steps
        
        # ========== Drone Parameters ==========
        self.MACHINES = 1  # Number of drones (single drone tracking)
        self.OBSTACLES = 0  # Number of static obstacles
        
        # Drone physical parameters
        self.DRONE_MASS = 1.5  # kg
        self.DRONE_MAX_VELOCITY = 3.0  # m/s
        self.DRONE_MAX_ACCELERATION = 2.0  # m/s^2
        self.DRONE_MAX_ANGULAR_VELOCITY = 1.5  # rad/s
        
        # Drone moment of inertia
        self.DRONE_INERTIA_XX = 0.02  # kg*m^2
        self.DRONE_INERTIA_YY = 0.02  # kg*m^2
        self.DRONE_INERTIA_ZZ = 0.04  # kg*m^2
        
        # ========== State Vector Definition ==========
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        self.STATE_SIZE = 12
        self.CONTROL_SIZE = 4  # [thrust, roll_cmd, pitch_cmd, yaw_rate_cmd]
        
        # State indices
        self.STATE_X = 0
        self.STATE_Y = 1
        self.STATE_Z = 2
        self.STATE_VX = 3
        self.STATE_VY = 4
        self.STATE_VZ = 5
        self.STATE_ROLL = 6
        self.STATE_PITCH = 7
        self.STATE_YAW = 8
        self.STATE_WX = 9
        self.STATE_WY = 10
        self.STATE_WZ = 11
        
        # ========== Cost Function Weights ==========
        # Position tracking weights - reduced to prevent oscillation
        self.W_POSITION = np.array([8.0, 8.0, 6.0])  # [x, y, z] - reduced to prevent aggressive tracking
        self.W_VELOCITY = np.array([5.0, 5.0, 4.0])    # [vx, vy, vz] - increased for smoother velocity tracking
        self.W_ATTITUDE = np.array([1.0, 1.0, 3.0])    # [roll, pitch, yaw] - reduced roll/pitch, moderate yaw
        self.W_ANGULAR_RATE = np.array([2.0, 2.0, 1.5]) # [wx, wy, wz] - increased for stability

        # Control effort weights - increased to penalize aggressive control
        self.W_CONTROL = np.array([0.5, 2.0, 2.0, 1.0])  # [thrust, roll, pitch, yaw_rate] - higher penalty for attitude commands

        # Person tracking specific weights - reduced to prevent oscillation
        self.W_TRACKING_DISTANCE = 4.0  # Weight for maintaining optimal tracking distance - reduced for smoother tracking
        self.W_CAMERA_ANGLE = 2.0       # Weight for keeping person in camera view - reduced to prevent aggressive yaw corrections
        self.W_SMOOTH_TRACKING = 8.0    # Weight for smooth tracking motion - increased for much smoother tracking
        
        # ========== Constraints ==========
        # State constraints
        self.STATE_MIN = np.array([
            -50.0, -50.0, 0.5,      # Position limits [x, y, z]
            -self.DRONE_MAX_VELOCITY, -self.DRONE_MAX_VELOCITY, -self.DRONE_MAX_VELOCITY,  # Velocity limits
            -math.pi/4, -math.pi/4, -math.pi,  # Attitude limits [roll, pitch, yaw]
            -self.DRONE_MAX_ANGULAR_VELOCITY, -self.DRONE_MAX_ANGULAR_VELOCITY, -self.DRONE_MAX_ANGULAR_VELOCITY  # Angular rate limits
        ])
        
        self.STATE_MAX = np.array([
            50.0, 50.0, 20.0,       # Position limits [x, y, z]
            self.DRONE_MAX_VELOCITY, self.DRONE_MAX_VELOCITY, self.DRONE_MAX_VELOCITY,   # Velocity limits
            math.pi/4, math.pi/4, math.pi,     # Attitude limits [roll, pitch, yaw]
            self.DRONE_MAX_ANGULAR_VELOCITY, self.DRONE_MAX_ANGULAR_VELOCITY, self.DRONE_MAX_ANGULAR_VELOCITY   # Angular rate limits
        ])
        
        # Control constraints
        self.CONTROL_MIN = np.array([
            0.0,          # Minimum thrust (hover thrust)
            -math.pi/6,   # Minimum roll command
            -math.pi/6,   # Minimum pitch command
            -1.0          # Minimum yaw rate command
        ])
        
        self.CONTROL_MAX = np.array([
            20.0,         # Maximum thrust
            math.pi/6,    # Maximum roll command
            math.pi/6,    # Maximum pitch command
            1.0           # Maximum yaw rate command
        ])
        
        # ========== Person Tracking Parameters ==========
        # Optimized for X3 drone with 30° downward camera tilt
        self.OPTIMAL_TRACKING_DISTANCE = 5.0  # Optimal distance for 30° tilt (meters)
        self.MIN_TRACKING_DISTANCE = 3.0      # Minimum safe distance for tilted camera
        self.MAX_TRACKING_DISTANCE = 12.0     # Extended max distance due to better visibility with tilt
        self.TRACKING_HEIGHT_OFFSET = 1.5     # Increased height offset for better downward view
        self.TRACKING_FIXED_ALTITUDE = 3.5    # Higher altitude to maximize 30° tilt coverage (m)
        self.BASE_TRACKING_ANGULAR_VELOCITY = 0.08  # Base orbit rate (rad/s) - much reduced for stability
        self.TRACKING_SPEED_GAIN = 0.05       # Gain from person speed to orbit rate - reduced for stability
        self.MAX_TRACKING_ANGULAR_VELOCITY = 0.2    # Cap orbit rate - reduced for stability
        self.TARGET_POSITION_SMOOTHING = 0.85       # 0=no smoothing, 1=full smoothing - increased for smoother tracking
        self.PERSON_POSITION_FILTER_ALPHA = 0.8     # Smoothing for detected person position - increased for smoother tracking
        
        # Camera parameters - updated for X3 drone configuration
        self.CAMERA_FOV_HORIZONTAL = 1.2        # 68.75 degrees horizontal FOV (from model.sdf)
        self.CAMERA_FOV_VERTICAL = 0.9          # Estimated 51.6 degrees vertical FOV
        self.CAMERA_TILT_ANGLE = -0.5236        # 30 degrees down tilt (from model.sdf)
        self.CAMERA_FORWARD_OFFSET = 0.2        # Camera 0.2m forward from drone center
        
        # ========== Environmental Parameters ==========
        self.GRAVITY = 9.81  # m/s^2
        self.AIR_DENSITY = 1.225  # kg/m^3
        
        # Wind parameters (can be updated dynamically)
        self.WIND_VELOCITY = np.array([0.0, 0.0, 0.0])  # [wx, wy, wz] m/s
        self.WIND_TURBULENCE = 0.1  # Wind turbulence factor
        
        # ========== Solver Parameters ==========
        self.MAX_ITERATIONS = 20  # Reduced further for real-time performance
        self.CONVERGENCE_TOLERANCE = 5e-3  # Relaxed for faster convergence and stability
        self.STEP_SIZE = 0.02  # Reduced to prevent oscillations in optimization
        self.REGULARIZATION = 1e-4  # Increased for better numerical stability
        
        # ========== ROS2 Topic Names ==========
        self.TOPIC_DRONE_STATE = '/X3/odometry'
        self.TOPIC_PERSON_DETECTIONS = '/person_detections'
        self.TOPIC_CONTROL_OUTPUT = '/drone/control/waypoint_command'
        self.TOPIC_WAYPOINT_CMD = '/drone/control/waypoint_command'
        self.TOPIC_ATTITUDE_CMD = '/drone/control/attitude_command'
        self.TOPIC_TRAJECTORY_VIS = '/drone/trajectory'
        self.TOPIC_TARGET_VIS = '/drone/target'
        self.TOPIC_STATUS = '/drone/controller/status'
        
        # ========== Logging and Debug ==========
        self.DEBUG_MODE = True
        self.LOG_LEVEL = 'INFO'  # DEBUG, INFO, WARN, ERROR
        self.PUBLISH_VISUALIZATION = True
        self.SAVE_TRAJECTORY_DATA = False

    def get_initial_state(self):
        """Get initial state vector"""
        return np.zeros(self.STATE_SIZE)
    
    def get_hover_control(self):
        """Get hover control input"""
        hover_thrust = self.DRONE_MASS * self.GRAVITY
        return np.array([hover_thrust, 0.0, 0.0, 0.0])
    
    def is_state_valid(self, state):
        """Check if state is within constraints"""
        return np.all(state >= self.STATE_MIN) and np.all(state <= self.STATE_MAX)
    
    def is_control_valid(self, control):
        """Check if control is within constraints"""
        return np.all(control >= self.CONTROL_MIN) and np.all(control <= self.CONTROL_MAX)
    
    def clip_state(self, state):
        """Clip state to constraints"""
        return np.clip(state, self.STATE_MIN, self.STATE_MAX)
    
    def clip_control(self, control):
        """Clip control to constraints"""
        return np.clip(control, self.CONTROL_MIN, self.CONTROL_MAX)

# Global configuration instance
nmpc_config = NMPCConfig()
