#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 12:37:56 2017

@author: juan
"""
import pdb
import math
from black_board_class import BlackBoard, bbo
import black_board as bb

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
        bbo.environment_status = True
        result = None
        bb.laser_scan()
        bb.right_status()
        bb.front_status()
        bb.left_status()
        if experiment.get_label() == 'e1':
            if bbo.Right1:
                bbo.adv_distance = bbo.distance_front - 1.7
            else:
                bbo.adv_distance = 0.0
            bbo.adv_angle = 0.0
            bb.move_adv()
            if not bbo.move_fail and bbo.Right1:
                bb.laser_scan()
                bb.right_status()
                bb.front_status()
                result = 'r1'  # moved forward following a wall on the right
            elif not bbo.move_fail:
                bb.laser_scan()
                bb.right_status()
                bb.front_status()
                result = 'r4'  # moving forward sensing no wall 
            else:
                bb.laser_scan()
                bb.right_status()
                bb.front_status()
                result = 'r10' # move failed: if robot bumps 
        elif experiment.get_label() == 'e2':
            #pdb.set_trace()
            bbo.adv_distance = 0.0
            bbo.adv_angle = math.pi/2
            bb.move_adv()
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            result = 'r2'   # turn left
        elif experiment.get_label() == 'e3':
            bbo.adv_distance = 0.0
            bbo.adv_angle = -math.pi/2
            bb.move_adv()
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            result = 'r3'   # turn right
        elif experiment.get_label() == 'e4':
            if bbo.driving_forward:
                bb.laser_scan()
                bb.right_status()
                bb.front_status()
                result = 'r4'  # front free: no wall
            else:
                bb.laser_scan()
                bb.right_status()
                result = 'r5'  # front busy: wall in front
        elif experiment.get_label() == 'e5':
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            if bbo.Right1:
                result = 'r6'   # right sensing: wall on the right
            else:
                result = 'r14'   # nothing on the right
        elif experiment.get_label() == 'e6':
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            bb.left_status()
            if bbo.Left1:
                result = 'r7'   # left sensing: wall on the left
            else:
                result = 'r13'   # nothing on the left

        self.last_result = result
        bbo.environment_status = False
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
        bbo.environment_status = True
        experiment = intended_interaction.get_label()[:2]
        result = None
        bb.laser_scan()
        bb.right_status()
        bb.front_status()
        bb.left_status()
        if experiment == 'e1':
            if bbo.Right1:
                bbo.adv_distance = bbo.distance_front - 1.7
            else:
                bbo.adv_distance = 0.0
            bbo.adv_angle = 0.0
            bb.move_adv()
            if not bbo.move_fail and bbo.Right1:
                bb.laser_scan()
                bb.right_status()
                bb.front_status()
                result = 'r1'   # moved forward following a wall on the right
            elif not bbo.move_fail:
                bb.laser_scan()
                bb.right_status()
                bb.front_status()
                result = 'r4'  # moving forward sensing no wall    
            else:
                bb.laser_scan()
                bb.right_status()
                bb.front_status()
                result = 'r10' # move failed: if robot bumps
        elif experiment == 'e2':
            bbo.adv_distance = 0.0
            bbo.adv_angle = math.pi/2
            bb.move_adv()
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            result = 'r2'   # turn left
        elif experiment == 'e3':
            bbo.adv_distance = 0.0
            bbo.adv_angle = -math.pi/2
            bb.move_adv()
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            result = 'r3'   # turn right
        elif experiment == 'e4':
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            if bbo.driving_forward:
                result = 'r4'  # front free: no wall
            else:
                result = 'r5'  # front busy: wall in front
        elif experiment == 'e5':
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            if bbo.Right1:
                result = 'r6'   # right sensing: wall on the right
            else:
                result = 'r14'   # nothing on the right
        elif experiment == 'e6':
            bb.laser_scan()
            bb.right_status()
            bb.front_status()
            bb.left_status()
            if bbo.Left1:
                result = 'r7'   # left sensing: wall on the left
            else:
                result = 'r13'   # nothing on the left
                
        enacted_interaction = experiment+result
        self.last_interaction = enacted_interaction
        bbo.environment_status = False
        return enacted_interaction
    