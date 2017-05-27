#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 12:37:56 2017

@author: juan
"""
import pdb
import math
from black_board_class import BlackBoard, black_board_object
import black_board

class Environment:
    """
    Class that implements the basic real-world environment.
    """
    def __init__(self):
        self.last_result = None

    def return_result(self, experiment):
        """
        Consult the world and return primitive result in response to 
        the experiment initiated by the agent.
        :param experiment: (Experiment) experiment issued by the agent
        :return: (str) result
        """
        result = None
        black_board.laser_scan()
        black_board.right_status()
        black_board.front_status()
        if experiment.get_label() == 'e1':
            if black_board_object.Right1:
                black_board_object.adv_distance = black_board_object.distance_front - 1.7
            else:
                black_board_object.adv_distance = 0.0
            black_board_object.adv_angle = 0.0
            black_board.move_adv()
            if not black_board_object.move_fail and black_board_object.Right1:
                black_board.laser_scan()
                black_board.right_status()
                black_board.front_status()
                result = 'r1'  # moved forward following a wall on the right
            elif not black_board_object.move_fail:
                black_board.laser_scan()
                black_board.right_status()
                black_board.front_status()
                result = 'r4'  # moving forward sensing no wall 
            else:
                black_board.laser_scan()
                black_board.right_status()
                black_board.front_status()
                result = 'r10' # move failed: if robot bumps 
        elif experiment.get_label() == 'e2':
            #pdb.set_trace()
            black_board_object.adv_distance = 0.0
            black_board_object.adv_angle = math.pi/2
            black_board.move_adv()
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            result = 'r2'   # turn left
        elif experiment.get_label() == 'e3':
            black_board_object.adv_distance = 0.0
            black_board_object.adv_angle = -math.pi/2
            black_board.move_adv()
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            result = 'r3'   # turn right
        elif experiment.get_label() == 'e4':
            if black_board_object.driving_forward:
                black_board.laser_scan()
                black_board.right_status()
                black_board.front_status()
                result = 'r4'  # front free: no wall
            else:
                black_board.laser_scan()
                black_board.right_status()
                result = 'r5'  # front busy: wall in front
        elif experiment.get_label() == 'e5':
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            if black_board_object.Right1:
                result = 'r6'   # right sensing: wall on the right
            else:
                result = 'r14'   # nothing on the right
        elif experiment.get_label() == 'e6':
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            black_board.left_status()
            if black_board_object.Left1:
                result = 'r7'   # left sensing: wall on the left
            else:
                result = 'r13'   # nothing on the left

        self.last_result = result
        return result

class ConstructiveEnvironment:
    """
    Class that implements constructive environment, in which interactions 
    are the basic primitives.
    """
    # TIMESTEP = 1
    def __init__(self):
        self.last_interaction = None

    def enact_primitive_interaction(self, intended_interaction):
        """
        Consult the world and return enacted interaction in response to 
        the agent's intended interaction.
        :param intended_interaction: (Interaction) interaction attempted by the agent
        :return: (Interaction) interaction actually enacted
        """
        #pdb.set_trace()
        
        experiment = intended_interaction.get_label()[:2]
        result = None
        black_board.laser_scan()
        black_board.right_status()
        black_board.front_status()
        if experiment == 'e1':
            if black_board_object.Right1:
                black_board_object.adv_distance = black_board_object.distance_front - 1.7
            else:
                black_board_object.adv_distance = 0.0
            black_board_object.adv_angle = 0.0
            black_board.move_adv()
            if not black_board_object.move_fail and black_board_object.Right1:
                black_board.laser_scan()
                black_board.right_status()
                black_board.front_status()
                result = 'r1'   # moved forward following a wall on the right
            elif not black_board_object.move_fail:
                black_board.laser_scan()
                black_board.right_status()
                black_board.front_status()
                result = 'r4'  # moving forward sensing no wall    
            else:
                black_board.laser_scan()
                black_board.right_status()
                black_board.front_status()
                result = 'r10' # move failed: if robot bumps
        elif experiment == 'e2':
            black_board_object.adv_distance = 0.0
            black_board_object.adv_angle = math.pi/2
            black_board.move_adv()
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            result = 'r2'   # turn left
        elif experiment == 'e3':
            black_board_object.adv_distance = 0.0
            black_board_object.adv_angle = -math.pi/2
            black_board.move_adv()
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            result = 'r3'   # turn right
        elif experiment == 'e4':
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            if black_board_object.driving_forward:
                result = 'r4'  # front free: no wall
            else:
                result = 'r5'  # front busy: wall in front
        elif experiment == 'e5':
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            if black_board_object.Right1:
                result = 'r6'   # right sensing: wall on the right
            else:
                result = 'r14'   # nothing on the right
        elif experiment == 'e6':
            black_board.laser_scan()
            black_board.right_status()
            black_board.front_status()
            black_board.left_status()
            if black_board_object.Left1:
                result = 'r7'   # left sensing: wall on the left
            else:
                result = 'r13'   # nothing on the left
                
        enacted_interaction = experiment+result
        self.last_interaction = enacted_interaction

        return enacted_interaction
    