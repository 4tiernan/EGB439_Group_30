#!/usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
import numpy as np
import time
from typing import Optional, Iterable


class PiBotSim(object):
    def __init__(self,
                 pose: np.ndarray = np.array([1, 1, 0], dtype=np.float64),
                 ax: Optional[Axes] = None,
                 dt: float = 0.05,
                 realtime: bool = False,
                 arena_size: float = 2.0,
                 ):
        """
        Parameters
        ----------
        pose       : Initial [x, y, theta] of the robot.
        ax         : Optional existing matplotlib Axes to draw on.
        dt         : Physics timestep in seconds (default 0.05 s = 20 Hz).
        realtime   : If True, simulation sleeps to match wall-clock time so you
                     can watch the robot move.  If False, runs as fast as possible
                     and shows the result afterwards.
        arena_size : Side length of the square arena in metres.
        """
        # ── Figure / axes ────────────────────────────────────────────────────
        if ax is None:
            self.fig, _ = plt.subplots(1, 1)
            self.axes = self.fig.axes[0]
        else:
            self.axes = ax
            self.fig = ax.get_figure()

        # ── Robot state ───────────────────────────────────────────────────────
        self.pose = pose.copy()          # [x, y, theta]  (dynamics centre)
        self.dt = dt
        self.realtime = realtime
        self.arena_size = arena_size

        self.v = 0.0                     # initial forward velocity  (m/s)
        self.w = 0.0                     # initial angular velocity  (rad/s)

        # ── Physical parameters ───────────────────────────────────────────────
        self.wheel_radius = 0.03         # metres  — REPLACE WITH REAL MEASUREMENT
        self.wheel_base   = 0.10         # metres  — REPLACE WITH REAL MEASUREMENT

        # ── Motor limits ──────────────────────────────────────────────────────
        self.max_motor_cmd   = 100       # absolute command magnitude
        self.deadzone        = 5         # commands below this are ignored
        self.max_linear_speed  = 0.5     # m/s   — tune to your robot
        self.max_angular_speed = 2.0     # rad/s — tune to your robot

        # ── Path history ──────────────────────────────────────────────────────
        self.path = [self.pose[:2].copy()]

        # ── Extended: localiser simulation ───────────────────────────────────
        self.localiser_pose  = self.pose.copy()
        self.localiser_rate  = 2.0       # Hz  (matches real hardware)
        self.localiser_timer = 0.0
        self.pose_offset     = 0.02      # 20 mm forward offset of LED

        # ── Duration command support ──────────────────────────────────────────
        self.command_duration = None
        self.command_timer    = 0.0

    # =========================================================================
    # Core simulation step
    # =========================================================================
    def step(self):

        '''
        Update the simulator one timestep using the internal state of the 
        simulated pibot.
        '''

        # Update x y theta based on current velocity and angular velocity states

        # Extract current state
        x, y, theta = self.pose

        # Update x, y, theta using unicycle kinematics
        x     += self.v * np.cos(theta) * self.dt
        y     += self.v * np.sin(theta) * self.dt
        theta += self.w * self.dt


        # Update pose and path history
        self.pose = np.array([x, y, theta])
        self.path.append(self.pose[:2].copy())
        print(f"Pose: [x]: {np.round(self.pose[0], 2)}, [y]: {np.round(self.pose[1], 2)}, [theta]: {np.round(self.pose[2], 2)}")

        # Localiser update at 2 Hz 
        self.localiser_timer += self.dt
        if self.localiser_timer >= 1.0 / self.localiser_rate:
            self.localiser_timer = 0.0
            x, y, theta = self.pose
            x_l = x + self.pose_offset * np.cos(theta)
            y_l = y + self.pose_offset * np.sin(theta)
            self.localiser_pose = np.array([x_l, y_l, theta])
        #print(f"Forward Velocity: {np.round(self.v, 2)}, Angular Velocity: {np.round(self.w, 2)}, Localiser pose: {np.round(self.localiser_pose, 2)}")

        if self.command_duration is not None:
            self.command_timer += self.dt
            if self.command_timer >= self.command_duration:
                self.stop()
                self.command_duration = None
        
        if self.realtime:
            time.sleep(self.dt)

    def simulate(self):
        while True:
            self.step()
            self.update_plot()
            if self.v == 0 and self.w == 0:
                break  # robot has auto-stopped


    def run_interactive(self):
        """Step the simulation one dt each time you press the right arrow key."""
        
        def on_key(event):
            if event.key == 'right':
                self.step()
                self.update_plot()
        
        self.fig.canvas.mpl_connect('key_press_event', on_key)
        self.update_plot()  # draw initial state
        plt.show()          # blocks here, events fire while window is open

    # Plotting
    def update_plot(self):

        '''
        Show the current state of the simulation. 

        Note: Instead of using plt.plot to make your plots, use self.axes.plot()
        instead.
        You may also prefer to use self.fig.canvas.start_event_loop(wait_time)
        instead of plt.pause(wait_time) to actually trigger the graphical update.
        '''

        ax = self.axes
        ax.clear()

        # Define arena boundary 
        border = plt.Polygon(
            [[0, 0], [self.arena_size, 0],
             [self.arena_size, self.arena_size], [0, self.arena_size]],
            closed=True, fill=False, edgecolor='black', linewidth=2
        )
        ax.add_patch(border)
        ax.set_xlim(-0.1, self.arena_size + 0.1)
        ax.set_ylim(-0.1, self.arena_size + 0.1)
        ax.set_aspect('equal')

        # --- Path ------------------------------------------------------------
        path = np.array(self.path)
        if len(path) > 1:
            ax.plot(path[:, 0], path[:, 1], 'b-', linewidth=1, label='Path')

        # Mark start
        ax.plot(path[0, 0], path[0, 1], 'go', markersize=6, label='Start')

        # --- Robot (triangle pointing in heading direction) ------------------
        x, y, theta = self.pose
        L = 0.08  # triangle size

        triangle = np.array([
            [ L,    0   ],
            [-L/2,  L/2 ],
            [-L/2, -L/2 ],
            [ L,    0   ],
        ])
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta),  np.cos(theta)]])
        tri = (R @ triangle.T).T + np.array([x, y])
        ax.fill(tri[:-1, 0], tri[:-1, 1], color='red', alpha=0.7)
        ax.plot(tri[:, 0], tri[:, 1], 'r-')

        # --- Localiser pose (small cross) ------------------------------------
        lx, ly, _ = self.localiser_pose
        ax.plot(lx, ly, 'k+', markersize=8, label='Localiser')

        # --- Labels ----------------------------------------------------------
        ax.set_title("PiBot Simulator")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend(loc='upper right', fontsize=8)

        plt.pause(0.001)  # allow the plot to update

    # Motion commands, not really used.
    def move(self, forward_vel, rotational_vel, duration=None):

        forward_vel    = np.clip(forward_vel, -self.max_linear_speed, self.max_linear_speed)
        rotational_vel = np.clip(rotational_vel, -self.max_angular_speed, self.max_angular_speed)

        v_left  = forward_vel - (rotational_vel * self.wheel_base / 2)
        v_right = forward_vel + (rotational_vel * self.wheel_base / 2)

        scale = self.max_motor_cmd / self.max_linear_speed
        left_cmd  = v_left  * scale
        right_cmd = v_right * scale

        # Negate left_cmd to counteract the hardware flip in setVelocity
        self.setVelocity(-left_cmd, right_cmd, duration=duration)

    def setVelocity(self, motor_left=0, motor_right=0, duration=None, acceleration_time=None):
        """
        Mirror of the real PiBot.setVelocity interface.
        Converts raw motor commands [-100, 100] to (v, w) via unicycle model.
        Commands inside the deadzone are treated as zero.
        """
        # Fix motor orientation — left motor is flipped on real hardware
        motor_left  = -motor_left

        # Apply deadzone
        if abs(motor_left)  < self.deadzone: motor_left  = 0
        if abs(motor_right) < self.deadzone: motor_right = 0

        # Clamp to motor limits
        motor_left  = np.clip(motor_left,  -self.max_motor_cmd, self.max_motor_cmd)
        motor_right = np.clip(motor_right, -self.max_motor_cmd, self.max_motor_cmd)

        # Scale to wheel angular speed (rad/s), then to linear wheel speed (m/s)
        # Assumes motor_cmd 100 => max_linear_speed at each wheel
        scale = self.max_linear_speed / self.max_motor_cmd # 0.5m/s / 100 = 0.005 m/s per motor command unit
        v_left  = motor_left  * scale 
        v_right = motor_right * scale

        # Unicycle: v = (vR + vL)/2,  w = (vR - vL) / wheel_base
        self.v = (v_right + v_left)  / 2.0
        self.w = (v_right - v_left)  / self.wheel_base

        if duration is not None:
            self.command_duration = duration
            self.command_timer    = 0.0
        else:
            self.command_duration = None

    def stop(self):
        self.v = 0.0
        self.w = 0.0

    
    # Localiser / pose

    def getLocalizerPose(self, group_number = 30) -> Iterable:
        """
        Returns the simulated localiser pose (updated at localiser_rate Hz).
        Includes the forward LED offset relative to the wheel-centre pose.
        Between updates this returns the same pose (no blocking).
        """

        return self.localiser_pose.copy()

    def resetPose(self):
        """Reset robot to home pose and clear the path history."""
        home_pose = np.array([1.0, 1.0, 0.0], dtype=np.float64)
        self.pose           = home_pose.copy()
        self.localiser_pose = home_pose.copy()
        self.localiser_timer = 0.0
        self.path = [home_pose[:2].copy()]   # ← clear old path

    # Stubs — hardware not simulated
    def resetEncoder(self):
        return 0, 0
    def getEncoders(self) -> Iterable:
        return 0, 0
    def getVoltage(self) -> float:
        return 7.2
    def getCurrent(self) -> float:
        return 0.6
    def setLED(self, number, state):
        return True
    def pulseLED(self, number, duration):
        return True
    def getDIP(self) -> int:
        return 0b10101
    def getButton(self) -> bool:
        return False
    def setLEDArray(self, value):
        return True
    def printfOLED(self, text, *args):
        return True
    def setScreen(self, screen):
        return True
    def getLocalizerImage(self) -> np.ndarray:
        return np.zeros((480, 640, 3), dtype=np.uint8)
    def getImage(self) -> np.ndarray:
        return np.zeros((480, 640, 3), dtype=np.uint8)



        
