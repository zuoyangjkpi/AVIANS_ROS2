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
        
        # Control history for warm start
        self.control_history = []
        for _ in range(self.config.N):
            self.control_history.append(self.config.get_hover_control())
        
        # Optimization state
        self.optimization_time = 0.0
        self.iterations_used = 0
        self.cost_value = 0.0
        
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
        """Update target position based on person location"""
        if not self.person_detected:
            return
        
        # Calculate optimal tracking position
        # Position drone at optimal distance and height relative to person
        person_pos = self.person_position
        
        # Calculate direction from person to current drone position
        drone_pos = self.current_state.data[self.config.STATE_X:self.config.STATE_Z+1]
        direction = drone_pos - person_pos
        direction[2] = 0  # Ignore vertical component for horizontal positioning
        
        if np.linalg.norm(direction) > 0:
            direction = direction / np.linalg.norm(direction)
        else:
            direction = np.array([1.0, 0.0, 0.0])  # Default direction
        
        # Set target position
        self.target_position = (person_pos + 
                              direction * self.config.OPTIMAL_TRACKING_DISTANCE)
        self.target_position[2] = person_pos[2] + self.config.TRACKING_HEIGHT_OFFSET
        
        # Predict person velocity for target velocity
        self.target_velocity = self.person_velocity.copy()
        self.target_velocity[2] = 0.0  # No vertical tracking velocity
    
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
        
        # Thrust in world frame
        thrust_world = R @ thrust_body
        
        # Acceleration (including gravity)
        gravity = np.array([0.0, 0.0, -self.config.GRAVITY])
        acceleration = thrust_world / self.config.DRONE_MASS + gravity
        
        # Add wind disturbance
        wind_acceleration = -0.1 * (vel - self.config.WIND_VELOCITY)
        acceleration += wind_acceleration
        
        state_dot[self.config.STATE_VX:self.config.STATE_VZ+1] = acceleration
        
        # Attitude dynamics (simplified)
        # Roll and pitch controlled by commands with time constant
        tau_att = 0.2  # Attitude time constant
        state_dot[self.config.STATE_ROLL] = (roll_cmd - att[0]) / tau_att
        state_dot[self.config.STATE_PITCH] = (pitch_cmd - att[1]) / tau_att
        state_dot[self.config.STATE_YAW] = yaw_rate_cmd
        
        # Angular velocity dynamics (simplified)
        tau_omega = 0.1  # Angular velocity time constant
        omega_cmd = np.array([roll_cmd/tau_att, pitch_cmd/tau_att, yaw_rate_cmd])
        state_dot[self.config.STATE_WX:self.config.STATE_WZ+1] = (omega_cmd - omega) / tau_omega
        
        return state_dot
    
    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Compute rotation matrix from Euler angles"""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        
        return R
    
    def runge_kutta_4(self, state: State, control: np.ndarray, dt: float) -> State:
        """4th order Runge-Kutta integration"""
        k1 = self.drone_dynamics(state, control)
        k2 = self.drone_dynamics(state + k1 * (dt/2), control)
        k3 = self.drone_dynamics(state + k2 * (dt/2), control)
        k4 = self.drone_dynamics(state + k3 * dt, control)
        
        return state + (k1 + 2*k2 + 2*k3 + k4) * (dt/6)
    
    def predict_trajectory(self, initial_state: State, 
                         control_sequence: List[np.ndarray]) -> List[State]:
        """Predict trajectory given control sequence"""
        trajectory = [initial_state.copy()]
        current_state = initial_state.copy()
        
        for i in range(self.N):
            if i < len(control_sequence):
                control = control_sequence[i]
            else:
                control = self.config.get_hover_control()
            
            # Integrate dynamics
            next_state = self.runge_kutta_4(current_state, control, self.dt)
            
            # Apply state constraints
            next_state.data = self.config.clip_state(next_state.data)
            
            trajectory.append(next_state)
            current_state = next_state
        
        return trajectory
    
    def compute_cost(self, trajectory: List[State], 
                    control_sequence: List[np.ndarray]) -> float:
        """Compute total cost for trajectory and control sequence"""
        total_cost = 0.0
        
        for i in range(len(trajectory)-1):
            state = trajectory[i]
            
            if i < len(control_sequence):
                control = control_sequence[i]
            else:
                control = self.config.get_hover_control()
            
            # State cost
            state_cost = self._compute_state_cost(state, i)
            
            # Control cost
            control_cost = self._compute_control_cost(control)
            
            total_cost += state_cost + control_cost
        
        # Terminal cost
        terminal_state = trajectory[-1]
        terminal_cost = self._compute_terminal_cost(terminal_state)
        total_cost += terminal_cost
        
        return total_cost
    
    def _compute_state_cost(self, state: State, step: int) -> float:
        """Compute state cost"""
        cost = 0.0
        
        # Extract state components
        pos = state.data[self.config.STATE_X:self.config.STATE_Z+1]
        vel = state.data[self.config.STATE_VX:self.config.STATE_VZ+1]
        att = state.data[self.config.STATE_ROLL:self.config.STATE_YAW+1]
        omega = state.data[self.config.STATE_WX:self.config.STATE_WZ+1]
        
        # Position tracking cost
        pos_error = pos - self.target_position
        cost += np.sum(self.config.W_POSITION * pos_error**2)
        
        # Velocity tracking cost
        vel_error = vel - self.target_velocity
        cost += np.sum(self.config.W_VELOCITY * vel_error**2)
        
        # Attitude cost (prefer level flight)
        att_target = np.array([0.0, 0.0, att[2]])  # Keep current yaw, level roll/pitch
        att_error = att - att_target
        cost += np.sum(self.config.W_ATTITUDE * att_error**2)
        
        # Angular rate cost (prefer smooth motion)
        cost += np.sum(self.config.W_ANGULAR_RATE * omega**2)
        
        # Person tracking specific costs
        if self.person_detected:
            cost += self._compute_tracking_cost(state)
        
        return cost
    
    def _compute_tracking_cost(self, state: State) -> float:
        """Compute person tracking specific cost"""
        cost = 0.0
        
        # Extract drone position
        drone_pos = state.data[self.config.STATE_X:self.config.STATE_Z+1]
        
        # Distance cost - penalize deviation from optimal tracking distance
        distance = np.linalg.norm(drone_pos[:2] - self.person_position[:2])
        distance_error = distance - self.config.OPTIMAL_TRACKING_DISTANCE
        cost += self.config.W_TRACKING_DISTANCE * distance_error**2
        
        # Camera angle cost - ensure person stays in camera view
        camera_cost = self._compute_camera_cost(state)
        cost += self.config.W_CAMERA_ANGLE * camera_cost
        
        # Smooth tracking cost - penalize rapid changes
        if len(self.control_history) > 1:
            smooth_cost = self._compute_smoothness_cost()
            cost += self.config.W_SMOOTH_TRACKING * smooth_cost
        
        return cost
    
    def _compute_camera_cost(self, state: State) -> float:
        """Compute cost for keeping person in camera view"""
        # Extract drone position and orientation
        drone_pos = state.data[self.config.STATE_X:self.config.STATE_Z+1]
        yaw = state.data[self.config.STATE_YAW]
        
        # Vector from drone to person
        to_person = self.person_position - drone_pos
        
        # Camera direction (assuming camera points forward and down)
        camera_direction = np.array([
            math.cos(yaw) * math.cos(self.config.CAMERA_TILT_ANGLE),
            math.sin(yaw) * math.cos(self.config.CAMERA_TILT_ANGLE),
            math.sin(self.config.CAMERA_TILT_ANGLE)
        ])
        
        # Angle between camera direction and person direction
        if np.linalg.norm(to_person) > 0 and np.linalg.norm(camera_direction) > 0:
            cos_angle = np.dot(to_person, camera_direction) / (
                np.linalg.norm(to_person) * np.linalg.norm(camera_direction))
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = math.acos(cos_angle)
            
            # Cost increases as angle deviates from center of FOV
            fov_cost = (angle / (self.config.CAMERA_FOV_HORIZONTAL / 2))**2
            return fov_cost
        
        return 0.0
    
    def _compute_smoothness_cost(self) -> float:
        """Compute cost for smooth control changes"""
        if len(self.control_history) < 2:
            return 0.0
        
        # Penalize rapid changes in control
        control_diff = self.control_history[-1] - self.control_history[-2]
        return np.sum(control_diff**2)
    
    def _compute_control_cost(self, control: np.ndarray) -> float:
        """Compute control effort cost"""
        # Normalize control inputs
        normalized_control = np.zeros_like(control)
        normalized_control[0] = (control[0] - self.config.DRONE_MASS * self.config.GRAVITY) / (self.config.DRONE_MASS * self.config.GRAVITY)
        normalized_control[1:] = control[1:] / (math.pi/6)  # Normalize angles
        
        return np.sum(self.config.W_CONTROL * normalized_control**2)
    
    def _compute_terminal_cost(self, state: State) -> float:
        """Compute terminal state cost"""
        # Higher weight on terminal state to ensure convergence
        return 2.0 * self._compute_state_cost(state, self.N)
    
    def optimize(self) -> Tuple[np.ndarray, dict]:
        """
        Main optimization function using gradient descent
        Returns: (optimal_control, info_dict)
        """
        start_time = time.time()
        
        # Initialize with previous solution (warm start)
        control_sequence = [ctrl.copy() for ctrl in self.control_history]
        
        best_cost = float('inf')
        best_control_sequence = control_sequence.copy()
        
        for iteration in range(self.max_iter):
            # Predict trajectory
            trajectory = self.predict_trajectory(self.current_state, control_sequence)
            
            # Compute cost
            current_cost = self.compute_cost(trajectory, control_sequence)
            
            if current_cost < best_cost:
                best_cost = current_cost
                best_control_sequence = [ctrl.copy() for ctrl in control_sequence]
            
            # Check convergence
            if iteration > 0 and abs(prev_cost - current_cost) < self.tolerance:
                break
            
            # Compute gradients and update control sequence
            control_sequence = self._gradient_update(control_sequence, trajectory)
            
            prev_cost = current_cost
        
        # Update control history
        self.control_history = best_control_sequence
        
        # Store optimization info
        self.optimization_time = time.time() - start_time
        self.iterations_used = iteration + 1
        self.cost_value = best_cost
        
        # Return first control input
        optimal_control = best_control_sequence[0] if best_control_sequence else self.config.get_hover_control()
        
        info = {
            'cost': best_cost,
            'iterations': self.iterations_used,
            'time': self.optimization_time,
            'trajectory': self.predict_trajectory(self.current_state, best_control_sequence)
        }
        
        return optimal_control, info
    
    def _gradient_update(self, control_sequence: List[np.ndarray], 
                        trajectory: List[State]) -> List[np.ndarray]:
        """Update control sequence using gradient descent"""
        new_control_sequence = []
        
        for i in range(len(control_sequence)):
            control = control_sequence[i]
            gradient = self._compute_control_gradient(control_sequence, trajectory, i)
            
            # Gradient descent update
            new_control = control - self.alpha * gradient
            
            # Apply control constraints
            new_control = self.config.clip_control(new_control)
            
            new_control_sequence.append(new_control)
        
        return new_control_sequence
    
    def _compute_control_gradient(self, control_sequence: List[np.ndarray], 
                                trajectory: List[State], control_index: int) -> np.ndarray:
        """Compute gradient of cost with respect to control input"""
        gradient = np.zeros(self.config.CONTROL_SIZE)
        epsilon = 1e-6
        
        # Finite difference approximation
        for j in range(self.config.CONTROL_SIZE):
            # Perturb control
            control_plus = control_sequence.copy()
            control_minus = control_sequence.copy()
            
            control_plus[control_index] = control_plus[control_index].copy()
            control_minus[control_index] = control_minus[control_index].copy()
            
            control_plus[control_index][j] += epsilon
            control_minus[control_index][j] -= epsilon
            
            # Compute costs
            traj_plus = self.predict_trajectory(self.current_state, control_plus)
            traj_minus = self.predict_trajectory(self.current_state, control_minus)
            
            cost_plus = self.compute_cost(traj_plus, control_plus)
            cost_minus = self.compute_cost(traj_minus, control_minus)
            
            # Finite difference gradient
            gradient[j] = (cost_plus - cost_minus) / (2 * epsilon)
        
        return gradient
    
    def get_status(self) -> dict:
        """Get controller status information"""
        return {
            'person_detected': self.person_detected,
            'person_position': self.person_position.tolist(),
            'target_position': self.target_position.tolist(),
            'current_position': self.current_state.data[self.config.STATE_X:self.config.STATE_Z+1].tolist(),
            'optimization_time': self.optimization_time,
            'iterations_used': self.iterations_used,
            'cost_value': self.cost_value,
            'tracking_distance': np.linalg.norm(
                self.current_state.data[self.config.STATE_X:self.config.STATE_Z+1][:2] - 
                self.person_position[:2]
            ) if self.person_detected else 0.0
        }

