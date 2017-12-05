# -*- coding: utf-8 -*-
"""
Created on Wed May 10 12:37:56 2017

@author: juan
"""
import pdb
import math
import numpy as np
from black_board_class import BlackBoard, bbo
import move_advance as ma
from decode import Decode
from fancy_prompts import bcolors

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
                label = 2             
                result = 'r02'  #  turn left <-- RIGHT-FRONT are busy
            elif bbo.singularity_selection == 1:
                label = 3
                result = 'r03'  #  turn right <-- nothing on the RIGHT
            elif bbo.singularity_selection == 5:
                label = 1
                result = 'r01'  # moved forward following a wall on the right
            elif bbo.singularity_selection == 3:
                label = 4
                result = 'r04'   # right sensing: corner on the right
            elif bbo.singularity_selection == 4:
                label = 5
                result = 'r05'   # door on the right
            else:
                label = 0
                result = 'r00' # move failed: if robot bumps 
                
        self.last_result = result
        decoded = Decode('e1' + result)
        translated = decoded.get_translation()
        print "\n"
        print bcolors.OKGREEN + "Result: " + result + str(translated) + bcolors.ENDC
        
        #pdb.set_trace()
        """
        Write the image and label on corresponding files. Both are autonomously
        obtained by the agent itself.
        """
        i_name = '/home/juan/catkin_ws/src/adaptor001/src/images'
        a = [x/5.0 for x in bbo.raw_kinect_scan]
            
        with open(i_name, 'a+') as i_file:
            i_file.write(str(a) + ' ')
            
        l_name = '/home/juan/catkin_ws/src/adaptor001/src/labels'
        with open(l_name, 'a+') as l_file:
            l_file.write(str(label) + ' ')
            
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
                label = 2
                result = 'r02'  #  turn left <-- RIGHT-FRONT are busy
            elif bbo.singularity_selection == 1:
                label = 3
                result = 'r03'  #  turn right <-- nothing on the RIGHT
            elif bbo.singularity_selection == 5:
                label = 1
                result = 'r01'  # moved forward following a wall on the right
            elif bbo.singularity_selection == 3:
                label = 4
                result = 'r04'   # right sensing: corner on the right
            elif bbo.singularity_selection == 4:
                label = 5
                result = 'r05'   # door on the right
            else:
                label = 0
                result = 'r00' # move failed: if robot bumps 
                
        enacted_interaction = experiment+result
        self.last_interaction = enacted_interaction
        decoded = Decode('e1' + result)
        translated = decoded.get_translation()
        print "\n"
        print bcolors.OKGREEN + "Result: " + result + str(translated) + bcolors.ENDC
        
        #pdb.set_trace()
        """
        Write the image and label on corresponding files. Both are autonomously
        obtained by the agent itself.
        """
        i_name = '/home/juan/catkin_ws/src/adaptor001/src/images'
        a = [x/5.0 for x in bbo.raw_kinect_scan]
            
        with open(i_name, 'a+') as i_file:
            i_file.write(str(a) + ' ')
            
        l_name = '/home/juan/catkin_ws/src/adaptor001/src/labels'
        with open(l_name, 'a+') as l_file:
            l_file.write(str(label) + ' ')
            
        bbo.environment_status = False
        return enacted_interaction
    