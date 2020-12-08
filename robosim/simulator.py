import os
from ctypes import *
from typing import Dict

import numpy as np
from numpy.ctypeslib import as_ctypes


robosim_lib = cdll.LoadLibrary(os.path.join(os.path.dirname(__file__),
                                            'librobosim_c.so'))

robosim_lib.newWorld.argtypes = None
robosim_lib.newWorld.restype = c_void_p
robosim_lib.delWorld.argtypes = [c_void_p]
robosim_lib.delWorld.restype = None
robosim_lib.step.argtypes = [c_void_p, c_void_p]
robosim_lib.step.restype = None
robosim_lib.getState.argtypes = [c_void_p, c_void_p]
robosim_lib.getState.restype = None
robosim_lib.getFieldParams.argtypes = [c_void_p, c_void_p]
robosim_lib.getFieldParams.restype = None
robosim_lib.getEpisodeTime.argtypes = [c_void_p]
robosim_lib.getEpisodeTime.restype = int
robosim_lib.getGoalsBlue.argtypes = [c_void_p]
robosim_lib.getGoalsBlue.restype = int
robosim_lib.getGoalsYellow.argtypes = [c_void_p]
robosim_lib.getGoalsYellow.restype = int
robosim_lib.replace.argtypes = [c_void_p, c_void_p, c_void_p, c_void_p]
robosim_lib.replace.restype = None
robosim_lib.replace_with_vel.argtypes = [
    c_void_p, c_void_p, c_void_p, c_void_p]
robosim_lib.replace_with_vel.restype = None


class SimulatorVSS():
    '''
    RoboSim VSS Simulator.
    Based on FiraSim. Transfers all the graphic functions
    to handle in python, whenever the user wants to.
    The SimulatorVSS uses a C++ interface to create the simulation
    and physics of the simulator.
    '''

    def __init__(self, field_type: int = 0,
                 n_robots_blue: int = 3,
                 n_robots_yellow: int = 3,
                 time_step_ms: int = 16,
                 ball_pos: np.ndarray = None,
                 blue_robots_pos: np.ndarray = None,
                 yellow_robots_pos: np.ndarray = None) -> None:
        '''
            Creates our Simulator object.

            Parameters
            ----------
            field_type : int
                The number corresponding to the type of the field
                0 - 3x3 field
                1 - 5x5 field

            n_robots_blue : int
                Number of blue robots

            n_robots_yellow : int
                Number of blue robots

            time_step_ms : int
                Simulation timestep in miliseconds

            ball_pos : np.ndarray
                Ball position array [ballX, ballY, ballVx, ballVy]

            yellow and blue robots_pos : np.ndarray
                Array of robot position array [[robotX, robotY, robotTheta]]

            Returns
            -------
            None

        '''
        self.field_type: int = field_type
        self.n_robots_blue: int = n_robots_blue
        self.n_robots_yellow: int = n_robots_yellow
        self.time_step_ms = time_step_ms
        # convert from list to np.ndarray
        ball_pos = np.array(ball_pos).flatten()
        blue_robots_pos = np.array(blue_robots_pos).flatten()
        yellow_robots_pos = np.array(yellow_robots_pos).flatten()
        self.world: object = robosim_lib.newWorld(self.field_type,
                                                  self.n_robots_blue,
                                                  self.n_robots_yellow,
                                                  self.time_step_ms,
                                                  as_ctypes(ball_pos),
                                                  as_ctypes(blue_robots_pos),
                                                  as_ctypes(yellow_robots_pos)
                                                  )
        self.field_params: Dict[str, np.float64] = self.get_field_params()
        self.state_size: int = 5 \
            + (self.n_robots_blue * 6)\
            + (self.n_robots_yellow * 6)

    def __del__(self):
        robosim_lib.delWorld(self.world)

    def get_state(self) -> np.ndarray:
        '''
        Returns the state array.
        State:
            - Ball: x, y, z, v_x, v_y
            - Robots_blue: x, y, theta, v_x, v_y, v_theta
            - Robots_yellow: x, y, theta, v_x, v_y, v_theta
            Units:
                x, y, z     -> meters
                theta       -> degrees (0, 360)
                v_x, v_y    -> meters/seconds
                v_theta     -> degrees/seconds

        State size: 5 + 6*n_robots_blue + 6*n_robots_yellow

        Parameters
        ----------
        None

        Returns
        -------
        np.ndarray
            State

        '''
        state = np.zeros(self.state_size, dtype=np.float64)
        robosim_lib.getState(self.world, as_ctypes(state))
        return state

    def step(self, action: np.ndarray) -> None:
        '''
        Steps the simulator given an action.

        Parameters
        ----------
        action: np.ndarray
            Action of shape (6, 2),
            2 wheels' speed for 6 robots.

        Returns
        -------
        None

        '''
        action = np.array(action, dtype=np.float64)
        action = action.flatten()
        robosim_lib.step(self.world, as_ctypes(action))

    def reset(self,
              ball_pos: np.ndarray,
              blue_robots_pos: np.ndarray,
              yellow_robots_pos: np.ndarray) -> np.ndarray:
        '''
        Resets the simulator and it's render view
        if it's in use.

        Parameters
        ----------
        None

        Returns
        -------
        np.ndarray
            State

        '''
        robosim_lib.delWorld(self.world)
        self.world = robosim_lib.newWorld(self.field_type,
                                          self.n_robots_blue,
                                          self.n_robots_yellow,
                                          self.time_step_ms,
                                          as_ctypes(ball_pos.flatten()),
                                          as_ctypes(blue_robots_pos.flatten()),
                                          as_ctypes(
                                              yellow_robots_pos.flatten())
                                          )
        return self.get_state()

    def get_field_params(self) -> Dict[str, float]:
        '''
        Returns the field parameters from the given
        field type.

        Parameters
        ----------
        None

        Returns
        -------
        dict
            field_width, field_length,
            penalty_width, penalty_length,
            goal_width

        '''
        params = np.zeros(6, dtype=np.float64)
        keys = ['field_width', 'field_length',
                'penalty_width', 'penalty_length', 'goal_width', 'goal_depth']
        robosim_lib.getFieldParams(self.world, as_ctypes(params))
        return {key: param for key, param in zip(keys, params)}

    def get_status(self) -> Dict[str, int]:
        '''
        Returns the game status.

        Parameters
        ----------
        None

        Returns
        -------
        dict
            goals_blue, goals_yellow, time

        '''
        status: Dict[str, int] = {
            'goals_blue': 0, 'goals_yellow': 0, 'time': 0}
        status['goals_blue'] = robosim_lib.getGoalsBlue(self.world)
        status['goals_yellow'] = robosim_lib.getGoalsYellow(self.world)
        status['time_ms'] = robosim_lib.getEpisodeTime(self.world)
        return status

    def replace(self, ball_pos: np.ndarray,
                blue_pos: np.ndarray, yellow_pos: np.ndarray):
        ball_pos = ball_pos.flatten()
        blue_pos = blue_pos.flatten()
        yellow_pos = yellow_pos.flatten()
        robosim_lib.replace(self.world, as_ctypes(ball_pos),
                            as_ctypes(blue_pos), as_ctypes(yellow_pos))

    def replace_with_vel(self, ball: np.ndarray,
                         blue_pos: np.ndarray, yellow_pos: np.ndarray):
        ball = ball.flatten()
        blue_pos = blue_pos.flatten()
        yellow_pos = yellow_pos.flatten()
        robosim_lib.replace_with_vel(self.world, as_ctypes(ball),
                                     as_ctypes(blue_pos),
                                     as_ctypes(yellow_pos))
