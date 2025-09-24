#!/usr/bin/env python3
"""
Nonlinear Model Predictive Controller for Drone Person Tracking
Based on blimp_nmpc_wrapper_node.py from AirshipMPC
Adapted for ROS2 Jazzy, Python 3.12, and quadrotor dynamics
"""

import math
import numpy as np
from typing import Tuple, List, Optional
import time

from .config import nmpc_config

class State:
    """State vector class for NMPC operations"""
    
    def __init__(self, data=None):
        if data is None:
            self.data = np.zeros(nmpc_config.STATE_SIZE)
        else:
            self.data = np.array(data, dtype=np.float64)
    
    def __getitem__(self, index):
        return self.data[index]
    
    def __setitem__(self, index, value):
        self.data[index] = value
    
    def __add__(self, other):
        if isinstance(other, State):
            return State(self.data + other.data)
        return State(self.data + other)
    
    def __sub__(self, other):
        if isinstance(other, State):
            return State(self.data - other.data)
        return State(self.data - other)
    
    def __mul__(self, scalar):
        return State(self.data * scalar)
    
    def __rmul__(self, scalar):
        return State(self.data * scalar)
    
    def __truediv__(self, scalar):
        return State(self.data / scalar)
    
    def copy(self):
        return State(self.data.copy())
    
    def norm(self):
        return np.linalg.norm(self.data)

class DroneNMPCController:
    """Nonlinear Model Predictive Controller for drone person tracking"""
    
    def __init__(self):
        self.config = nmpc_config
        
        # Current state
        self.current_state = State()
        self.target_position = np.array([0.0, 0.0, 2.0])
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        
        # Person tracking state
        self.person_position = np.array([0.0, 0.0, 0.0])
        self.person_velocity = np.array([0.0, 0.0, 0.0])
        self.person_detected = False
        self.last_detection_time = 0.0
        self.tracking_height_offset = nmpc_config.TRACKING_HEIGHT_OFFSET
        self._desired_phase = 0.0
        
        # Control history for warm start
        self.control_history = []
        for _ in range(self.config.N):
            self.control_history.append(self.config.get_hover_control())
        
        # Optimization state
        self.optimization_time = 0.0
        self.iterations_used = 0
        self.cost_value = 0.0

        # Internal smoothing state
        self._last_target_position = None
        self._last_target_velocity = None

        # Initialize solver parameters
        self._init_solver_parameters()
    
    def _init_solver_parameters(self):
        """Initialize solver parameters"""
        self.dt = self.config.TIMESTEP
        self.N = self.config.N
        
        # Gradient descent parameters
        self.alpha = self.config.STEP_SIZE
        self.max_iter = self.config.MAX_ITERATIONS
        self.tolerance = self.config.CONVERGENCE_TOLERANCE
        self.regularization = self.config.REGULARIZATION
    
    def set_drone_state(self, position: np.ndarray, velocity: np.ndarray, 
                       orientation: np.ndarray, angular_velocity: np.ndarray):
        """Set current drone state"""
        self.current_state[self.config.STATE_X:self.config.STATE_Z+1] = position
        self.current_state[self.config.STATE_VX:self.config.STATE_VZ+1] = velocity
        self.current_state[self.config.STATE_ROLL:self.config.STATE_YAW+1] = orientation
        self.current_state[self.config.STATE_WX:self.config.STATE_WZ+1] = angular_velocity
    
    def set_person_detection(self, position: np.ndarray, velocity: np.ndarray = None):
        """Set detected person position and velocity"""
        self.person_position = position.copy()
        if velocity is not None:
            self.person_velocity = velocity.copy()
        else:
            self.person_velocity = np.array([0.0, 0.0, 0.0])
        
        self.person_detected = True
        self.last_detection_time = time.time()
        
        # Update target position for tracking
        self._update_tracking_target()

    def _update_tracking_target(self):
        """Update target position based on person location with circular tracking behavior"""
        if not self.person_detected:
            return
        
        person_pos = self.person_position
        
        # Circular tracking behavior - drone orbits around the person
        # Calculate current phase angle relative to person
        drone_pos = self.current_state.data[self.config.STATE_X:self.config.STATE_Z+1]
        relative_pos = drone_pos[:2] - person_pos[:2]
        
        if np.linalg.norm(relative_pos) > 0.1:  # Avoid singularity
            current_phase = math.atan2(relative_pos[1], relative_pos[0])
        else:
            current_phase = self._desired_phase if hasattr(self, '_desired_phase') else 0.0
        
        # Desired phase evolution - slowly orbit around the person with damping
        person_speed = np.linalg.norm(self.person_velocity[:2])
        base_angular_velocity = self.config.BASE_TRACKING_ANGULAR_VELOCITY
        adaptive_angular_velocity = base_angular_velocity + \
            self.config.TRACKING_SPEED_GAIN * person_speed
        adaptive_angular_velocity = min(adaptive_angular_velocity,
                                        self.config.MAX_TRACKING_ANGULAR_VELOCITY)

        # Add damping based on distance to target position to prevent oscillations
        target_distance = np.linalg.norm(drone_pos[:2] - person_pos[:2])

        # Enhanced damping for tilted camera tracking
        if target_distance > self.config.OPTIMAL_TRACKING_DISTANCE:
            # Stronger damping for X3's optimal tracking distance (5.0m)
            damping_factor = max(0.2, 1.0 - (target_distance - self.config.OPTIMAL_TRACKING_DISTANCE) / 3.0)
            adaptive_angular_velocity *= damping_factor

        # Additional damping based on camera tilt compensation
        # For 30° tilt, reduce angular velocity when person is very close to prevent oscillation
        if target_distance < 4.0:  # Within close range for tilted camera
            tilt_damping = 0.7 + 0.3 * (target_distance / 4.0)  # 70%-100% depending on distance
            adaptive_angular_velocity *= tilt_damping

        dt = self.config.TIMESTEP
        if not hasattr(self, '_desired_phase'):
            # Initialize phase when first detected
            self._desired_phase = current_phase
        else:
            self._desired_phase += adaptive_angular_velocity * dt

        # Calculate target position on the circle
        radius = self.config.OPTIMAL_TRACKING_DISTANCE
        target_x = person_pos[0] + radius * math.cos(self._desired_phase)
        target_y = person_pos[1] + radius * math.sin(self._desired_phase)
        target_z = self.config.TRACKING_FIXED_ALTITUDE

        new_target = np.array([target_x, target_y, target_z])
        if self._last_target_position is not None:
            smoothing = np.clip(self.config.TARGET_POSITION_SMOOTHING, 0.0, 0.99)
            new_target = smoothing * new_target + (1.0 - smoothing) * self._last_target_position
        self.target_position = new_target
        self._last_target_position = new_target.copy()
        
        # Calculate target velocity for smooth circular motion
        # Tangential velocity = radius * angular_velocity
        tangential_speed = radius * adaptive_angular_velocity
        tangent_x = -math.sin(self._desired_phase) * tangential_speed
        tangent_y = math.cos(self._desired_phase) * tangential_speed
        
        # Add person's velocity to follow the moving center (disabled for now)
        new_velocity = np.zeros(3)
        new_velocity[0] += tangent_x
        new_velocity[1] += tangent_y
        new_velocity[2] = 0.0  # No vertical tracking velocity

        if self._last_target_velocity is not None:
            smoothing = np.clip(self.config.TARGET_POSITION_SMOOTHING, 0.0, 0.99)
            new_velocity = smoothing * new_velocity + (1.0 - smoothing) * self._last_target_velocity
        self.target_velocity = new_velocity
        self._last_target_velocity = new_velocity.copy()
        
        # Ensure drone always faces the person (yaw control)
        to_person = person_pos - np.array([target_x, target_y, target_z])
        desired_yaw = math.atan2(to_person[1], to_person[0]) + math.pi  # 加上pi确保面向人

        # Store desired yaw for use in cost function
        self._desired_yaw = desired_yaw

    def reset_phase(self, phase: float):
        self._desired_phase = phase

    def clear_detection(self):
        self.person_detected = False
        self._last_target_position = None
        self._last_target_velocity = None
        self.person_velocity = np.zeros(3)

    def set_tracking_height_offset(self, offset: float):
        self.tracking_height_offset = offset
    
    def set_fixed_altitude(self, altitude: float):
        self.config.TRACKING_FIXED_ALTITUDE = altitude
    
    def drone_dynamics(self, state: State, control: np.ndarray) -> State:
        """Quadrotor dynamics model"""
        # Extract state variables
        pos = state.data[self.config.STATE_X:self.config.STATE_Z+1]
        vel = state.data[self.config.STATE_VX:self.config.STATE_VZ+1]
        att = state.data[self.config.STATE_ROLL:self.config.STATE_YAW+1]
        omega = state.data[self.config.STATE_WX:self.config.STATE_WZ+1]
        
        # Extract control inputs
        thrust = control[0]
        roll_cmd = control[1]
        pitch_cmd = control[2]
        yaw_rate_cmd = control[3]
        
        # State derivative
        state_dot = State()
        
        # Position dynamics: dx/dt = v
        state_dot[self.config.STATE_X:self.config.STATE_Z+1] = vel
        
        # Velocity dynamics: dv/dt = a
        # Rotation matrix from body to world frame
        R = self._rotation_matrix(att[0], att[1], att[2])
        
        # Thrust vector in body frame
        thrust_body = np.array([0.0, 0.0, thrust])
        
        # Gravity in world frame
        gravity = np.array([0.0, 0.0, -self.config.GRAVITY * self.config.DRONE_MASS])
        
        # Total force in world frame
        force_world = R @ thrust_body + gravity
        
        # Acceleration = Force / Mass
        acc = force_world / self.config.DRONE_MASS
        
        state_dot[self.config.STATE_VX:self.config.STATE_VZ+1] = acc
        
        # Attitude dynamics (using proper rotational kinematics)
        # Convert body rates to Euler angle rates
        phi, theta, psi = att[0], att[1], att[2]
        
        # Avoid singularity at 90 degrees pitch
        cos_theta = math.cos(theta)
        tan_theta = math.tan(theta) if abs(cos_theta) > 1e-6 else 0.0
        
        p, q, r = omega[0], omega[1], omega[2]
        
        # Euler angle rates
        phi_dot = p + q * math.sin(phi) * tan_theta + r * math.cos(phi) * tan_theta
        theta_dot = q * math.cos(phi) - r * math.sin(phi)
        psi_dot = q * math.sin(phi) / cos_theta + r * math.cos(phi) / cos_theta
        
        state_dot[self.config.STATE_ROLL] = phi_dot
        state_dot[self.config.STATE_PITCH] = theta_dot
        state_dot[self.config.STATE_YAW] = psi_dot
        
        # Angular velocity dynamics (PD control for angular rates)
        # Simplified model with damping
        Ixx = self.config.DRONE_INERTIA_XX  # kg*m^2 (approximate moment of inertia)
        Iyy = self.config.DRONE_INERTIA_YY
        Izz = self.config.DRONE_INERTIA_ZZ
        
        # Control inputs to moments (simplified)
        tau_roll = 0.1 * (roll_cmd - att[0]) - 0.05 * omega[0]   # PD control
        tau_pitch = 0.1 * (pitch_cmd - att[1]) - 0.05 * omega[1]  # PD control
        tau_yaw = 0.1 * yaw_rate_cmd - 0.02 * omega[2]  # PD control for yaw rate
        
        # Euler's rotation equations
        omega_dot = np.array([
            (Iyy - Izz) / Ixx * omega[1] * omega[2] + tau_roll / Ixx,
            (Izz - Ixx) / Iyy * omega[0] * omega[2] + tau_pitch / Iyy,
            (Ixx - Iyy) / Izz * omega[0] * omega[1] + tau_yaw / Izz
        ])
        
        state_dot[self.config.STATE_WX:self.config.STATE_WZ+1] = omega_dot
        
        return state_dot
    
    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Calculate rotation matrix from Euler angles"""
        # Rotation matrices
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        
        # Combined rotation matrix
        return Rz @ Ry @ Rx
    
    def predict_trajectory(self, initial_state: State, controls: List[np.ndarray]) -> List[State]:
        """Predict future trajectory using Euler integration"""
        trajectory = [initial_state.copy()]
        current_state = initial_state.copy()
        
        for i in range(self.N):
            # Get control input
            control = controls[i] if i < len(controls) else self.config.get_hover_control()
            
            # Integrate dynamics using Euler method
            state_dot = self.drone_dynamics(current_state, control)
            current_state = current_state + state_dot * self.dt
            
            # Clip state to constraints
            current_state.data = self.config.clip_state(current_state.data)
            
            trajectory.append(current_state.copy())
        
        return trajectory
    
    def compute_stage_cost(self, state: State, control: np.ndarray, time_step: int) -> float:
        """Compute stage cost for a given state and control"""
        cost = 0.0
        
        # Position tracking cost
        pos_error = state.data[self.config.STATE_X:self.config.STATE_Z+1] - self.target_position
        cost += np.sum(self.config.W_POSITION * pos_error**2)
        
        # Velocity tracking cost
        vel_error = state.data[self.config.STATE_VX:self.config.STATE_VZ+1] - self.target_velocity
        cost += np.sum(self.config.W_VELOCITY * vel_error**2)
        
        # Attitude cost (maintain level flight except for yaw)
        att_error = state.data[self.config.STATE_ROLL:self.config.STATE_YAW]  # 不包括yaw
        cost += np.sum(self.config.W_ATTITUDE[:2] * att_error**2)
        
        # Yaw tracking cost
        if hasattr(self, '_desired_yaw'):
            yaw_error = self._wrap_angle(state[self.config.STATE_YAW] - self._desired_yaw)
            cost += self.config.W_ATTITUDE[2] * yaw_error**2
        
        # Angular rate cost
        omega_error = state.data[self.config.STATE_WX:self.config.STATE_WZ+1]
        cost += np.sum(self.config.W_ANGULAR_RATE * omega_error**2)
        
        # Control effort cost
        cost += np.sum(self.config.W_CONTROL * control**2)
        
        # Person tracking specific costs
        if self.person_detected:
            # Distance to person cost
            drone_pos = state.data[self.config.STATE_X:self.config.STATE_Z+1]
            distance_to_person = np.linalg.norm(drone_pos - self.person_position)
            distance_error = abs(distance_to_person - self.config.OPTIMAL_TRACKING_DISTANCE)
            cost += self.config.W_TRACKING_DISTANCE * distance_error**2
            
            # Camera angle cost (keep person in view)
            to_person = self.person_position - drone_pos
            if np.linalg.norm(to_person) > 0.1:
                # 计算无人机朝向
                yaw = state[self.config.STATE_YAW]
                drone_facing = np.array([math.cos(yaw), math.sin(yaw)])
                
                # 计算到人的方向
                to_person_dir = to_person[:2] / np.linalg.norm(to_person[:2])
                
                # 计算夹角
                dot_product = np.dot(drone_facing, to_person_dir)
                angle_error = math.acos(np.clip(dot_product, -1.0, 1.0))
                cost += self.config.W_CAMERA_ANGLE * angle_error**2
            
            # Smooth tracking cost - penalize large changes in target position
            if self._last_target_position is not None:
                target_change = np.linalg.norm(self.target_position - self._last_target_position)
                cost += self.config.W_SMOOTH_TRACKING * target_change**2
        
        return cost
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def compute_total_cost(self, trajectory: List[State], controls: List[np.ndarray]) -> float:
        """Compute total cost over trajectory"""
        total_cost = 0.0
        
        for i in range(len(controls)):
            stage_cost = self.compute_stage_cost(trajectory[i], controls[i], i)
            total_cost += stage_cost
        
        return total_cost
    
    def compute_gradient(self, controls: List[np.ndarray]) -> List[np.ndarray]:
        """Compute gradient of cost function with respect to controls"""
        gradient = []
        
        # Predict current trajectory
        trajectory = self.predict_trajectory(self.current_state, controls)
        current_cost = self.compute_total_cost(trajectory, controls)
        
        # Compute gradient using finite differences
        for i in range(len(controls)):
            control_grad = np.zeros_like(controls[i])
            
            for j in range(len(controls[i])):
                # Perturb control
                perturbed_controls = [ctrl.copy() for ctrl in controls]
                perturbed_controls[i][j] += self.regularization
                
                # Predict perturbed trajectory
                perturbed_trajectory = self.predict_trajectory(self.current_state, perturbed_controls)
                perturbed_cost = self.compute_total_cost(perturbed_trajectory, perturbed_controls)
                
                # Compute gradient component
                control_grad[j] = (perturbed_cost - current_cost) / self.regularization
            
            gradient.append(control_grad)
        
        return gradient
    
    def optimize(self) -> Tuple[np.ndarray, dict]:
        """Optimize control inputs using gradient descent"""
        start_time = time.time()
        
        # Initialize controls (use previous solution as warm start)
        controls = [ctrl.copy() for ctrl in self.control_history]
        
        # Gradient descent optimization
        for iteration in range(self.max_iter):
            # Compute gradient
            gradient = self.compute_gradient(controls)
            
            # Update controls
            for i in range(len(controls)):
                controls[i] -= self.alpha * gradient[i]
                # Clip to constraints
                controls[i] = self.config.clip_control(controls[i])
            
            # Check for convergence
            grad_norm = np.linalg.norm(np.concatenate(gradient))
            if grad_norm < self.tolerance:
                break
        
        # Store solution for warm start
        self.control_history = controls
        
        # Extract first control input as optimal control
        optimal_control = controls[0] if controls else self.config.get_hover_control()
        
        # Predict trajectory for visualization
        trajectory = self.predict_trajectory(self.current_state, controls)
        
        # Store optimization info
        self.optimization_time = time.time() - start_time
        self.iterations_used = iteration + 1
        self.cost_value = self.compute_total_cost(trajectory, controls)
        
        info = {
            'trajectory': [s.data for s in trajectory],
            'iterations': self.iterations_used,
            'optimization_time': self.optimization_time,
            'cost': self.cost_value
        }
        
        return optimal_control, info
    
    def get_status(self) -> dict:
        """Get controller status"""
        status = {
            'person_detected': self.person_detected,
            'tracking_distance': np.linalg.norm(
                self.current_state.data[self.config.STATE_X:self.config.STATE_Z+1] - self.person_position
            ) if self.person_detected else 0.0,
            'optimization_time': self.optimization_time,
            'iterations_used': float(self.iterations_used),
            'cost_value': self.cost_value
        }
        
        return status
