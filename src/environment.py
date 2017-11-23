#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 12:37:56 2017

@author: juan
"""
import pdb
import math
from black_board_class import BlackBoard, bbo
import move_advance as ma

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
        if experiment.get_label() == 'e1':
            if True in bbo.Right:
                bbo.adv_distance = bbo.distance_front - 1.7
            else:
                bbo.adv_distance = 0.0
            bbo.adv_angle = 0.0
            ma.move_adv()
            if bbo.singularity_selection == 2:
               result = 'r02'  #  turn left <-- RIGHT-FRONT are busy
            elif bbo.singularity_selection == 1:
               result = 'r03'  #  turn right <-- nothing on the RIGHT
            elif bbo.singularity_selection == 5:
                result = 'r01'  # moved forward following a wall on the right
            elif bbo.singularity_selection == 3:
                result = 'r06'   # right sensing: corner on the right
            elif bbo.singularity_selection == 4:
                result = 'r08'   # door on the right
            else:
                result = 'r10' # move failed: if robot bumps 
#        elif experiment.get_label() == 'e2':
#            #pdb.set_trace()
#            bbo.adv_distance = 0.0
#            bbo.adv_angle = math.pi/2
#            ma.move_adv()
#            if not bbo.move_fail and True in bbo.Right:
#                result = 'r2'   # turn left sensing a wall on the right
#            elif not bbo.move_fail and not (True in bbo.Right):
#                result = 'r15'  #  turn left sensing no wall 
#            else:
#                result = 'r16' # turn left failed: if robot bumps 
#        elif experiment.get_label() == 'e3':
#            bbo.adv_distance = 0.0
#            bbo.adv_angle = -math.pi/2
#            ma.move_adv()
#            if not bbo.move_fail and True in bbo.Right:
#                result = 'r3'   # turn right sensing a wall on the right
#            elif not bbo.move_fail and not (True in bbo.Right):
#                result = 'r17'  #  turn right sensing no wall 
#            else:
#                result = 'r18' # turn right failed: if robot bumps 
#        elif experiment.get_label() == 'e4':
#            if bbo.driving_forward:
#                result = 'r4'  # front free: no wall
#            else:
#                result = 'r5'  # front busy: wall in front
        
#        elif experiment.get_label() == 'e6':
#            if True in bbo.Left:
#                result = 'r7'   # left sensing: wall on the left
#            else:
#                result = 'r13'   # nothing on the left

        self.last_result = result
        pdb.set_trace()
        bbo.labels.write(result + " ")
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
        if experiment == 'e1':
            if True in bbo.Right:
                bbo.adv_distance = bbo.distance_front - 1.7
            else:
                bbo.adv_distance = 0.0
            bbo.adv_angle = 0.0
            ma.move_adv()
            if bbo.singularity_selection == 2:
                result = 'r02'  #  turn left <-- RIGHT-FRONT are busy
            elif bbo.singularity_selection == 1:
                result = 'r03'  #  turn right <-- nothing on the RIGHT
            elif bbo.singularity_selection == 5:
                result = 'r01'  # moved forward following a wall on the right
            elif bbo.singularity_selection == 3:
                result = 'r06'   # right sensing: corner on the right
            elif bbo.singularity_selection == 4:
                result = 'r08'   # door on the right
            else:
                result = 'r10' # move failed: if robot bumps 
#            if not bbo.move_fail and True in bbo.Right:
#                result = 'r1'   # moved forward following a wall on the right
#            elif not bbo.move_fail:
#                result = 'r4'  # moving forward sensing no wall    
#            else:
#                result = 'r10' # move failed: if robot bumps
#        elif experiment == 'e2':
#            bbo.adv_distance = 0.0
#            bbo.adv_angle = math.pi/2
#            ma.move_adv()
#            if not bbo.move_fail and True in bbo.Right:
#                result = 'r2'   # turn left sensing a wall on the right
#            elif not bbo.move_fail and not (True in bbo.Right):
#                result = 'r15'  #  turn left sensing no wall 
#            else:
#                result = 'r16' # turn left failed: if robot bumps
#        elif experiment == 'e3':
#            bbo.adv_distance = 0.0
#            bbo.adv_angle = -math.pi/2
#            ma.move_adv()
#            if not bbo.move_fail and True in bbo.Right:
#                result = 'r3'   # turn right sensing a wall on the right
#            elif not bbo.move_fail and not (True in bbo.Right):
#                result = 'r17'  #  turn right sensing no wall 
#            else:
#                result = 'r18' # turn right failed: if robot bumps 
#        elif experiment == 'e4':
#            if bbo.driving_forward:
#                result = 'r4'  # front free: no wall
#            else:
#                result = 'r5'  # front busy: wall in front
#        elif experiment == 'e5':
#            if True in bbo.Right:
#                result = 'r6'   # right sensing: wall on the right
#            else:
#                result = 'r14'   # nothing on the right
#        elif experiment == 'e6':
#            if True in bbo.Left:
#                result = 'r7'   # left sensing: wall on the left
#            else:
#                result = 'r13'   # nothing on the left
                
        enacted_interaction = experiment+result
        self.last_interaction = enacted_interaction
        bbo.labels.write(result + " ")
        bbo.environment_status = False
        return enacted_interaction
    