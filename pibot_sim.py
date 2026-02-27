#!/usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
import numpy as np
import sys
import time
from typing import Optional, Iterable

import cv2


class PiBotSim(object):
    def __init__(self, pose:np.ndarray = np.array([1,1,0],dtype=np.float64), ax:Optional[Axes] = None, dt:float=0.05,):
        # Initialise figure for plotting.
        if ax is None:
            self.fig,_ = plt.subplots(1,1)
            self.axes = self.fig.axes[0]
        else:
            self.axes = ax
            self.fig = ax.get_figure()

  
    def step(self):
        '''
        Update the simulator one timestep using the internal state of the 
        simulated pibot.
        '''
        raise NotImplementedError
    
    def update_plot(self):
        '''
        Show the current state of the simulation. 

        Note: Instead of using plt.plot to make your plots, use self.axes.plot()
        instead.
        You may also prefer to use self.fig.canvas.start_event_loop(wait_time)
        instead of plt.pause(wait_time) to actually trigger the graphical update.
        '''
        
        raise NotImplementedError

    
    def move(self,forward_vel:float,rotational_vel:float):
        '''
        Modifies the internal state of the robot to reflect the desired velocity
        and rotational velocity.
        '''
        raise NotImplementedError

    def stop(self):
        '''
        Stop the robot.
        '''
        raise NotImplementedError


    def getLocalizerPose(self) -> Iterable:
        '''
        Get an estimate of the robot's pose.

        Returns
        -------
        pose - Iterable: The current pose of the robot according to the localiser.
        '''
        raise NotImplementedError
    
    def setVelocity(self, motor_left=0, motor_right=0, duration=None, acceleration_time=None):
        raise NotImplementedError
    
    def resetPose(self):
        '''
        Reset the robot's pose. Choose a hard-coded home pose for your robot.
        '''
        raise NotImplementedError
    
    def resetEncoder(self):
        '''
        Reset the robot's encoder values to (0,0)
        '''
        raise NotImplementedError

    def getEncoders(self) -> Iterable:
        '''
        Returns the current encoder values
        '''
        raise NotImplementedError

    def getVoltage(self) -> float:
        '''
        Returns the current voltage of the battery
        '''
        raise NotImplementedError
    
    def getCurrent(self) -> float:
        '''
        Returns the current the battery is currently providing.
        '''
        raise NotImplementedError
    
    def setLED(self, number, state):
        '''
        Turns on or off a particular LED
        '''
        raise NotImplementedError

    def pulseLED(self, number, duration):
        '''
        Pulses a particular LED on or off
        '''
        raise NotImplementedError

    def getDIP(self) -> int:
        '''
        Returns the state of the DIP switch binary-encoded.
        '''
        raise NotImplementedError
    
    def getButton(self) -> bool:
        '''
        Returns the state of the user button.
        '''
        raise NotImplementedError
   
    def setLEDArray(self, value):
        '''
        Sets the state of the blue LEDs on top of the robot.
        '''
        raise NotImplementedError

    def printfOLED(self, text, *args):
        '''
        Prints user text to the OLED screen
        '''
        raise NotImplementedError

    def setScreen(self, screen):
        '''
        Sets which page is currently being viewed on the OLED screen. 
        '''
        raise NotImplementedError

    def getLocalizerImage(self) -> np.ndarray:
        '''
        Returns the image the localiser used to determine robot pose.
        '''
        raise NotImplementedError
    
    def getImage(self) -> np.ndarray:
        '''
        Returns an image from the onboard RGB camera.
        '''
        raise NotImplementedError