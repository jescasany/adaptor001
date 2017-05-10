#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 02 2017
Last visited on 18/04/2017
@author: juan

The Enactive Cognitive Architecture (ECA)
based on the paper of Georgeon, Marshall, and Manzotti (2013). ECA: An enactivist cognitive
architecture based on sensorimotor modeling. Biologically Inspired Cognitive
Architectures, 6:46-57.

Implemented following the Behavior Trees model and the enactive agents code from Katja Abramova.
"""
__author__ = 'juan'

import pdb

import rospy

from geometry_msgs.msg import Twist

from pi_trees_ros.pi_trees_ros import *

from advance import *
import black_board
from black_board_class import BlackBoard, black_board_object

from fancy_prompts import bcolors
from decode import Decode
from environment import Environment, ConstructiveEnvironment
from interaction import Interaction
from boredom import *

import argparse
#from json import loads, dumps
from collections import OrderedDict
from experiment import Experiment, RecursiveExperiment
from result import Result
from anticipation import Anticipation, RecursiveAnticipation, ConstructiveAnticipation
import random


class EcaAgent02:
    INTERACTION_ENACTION_HISTORY_SIZE = 50
    def __init__(self):
        #pdb.set_trace()
        rospy.init_node("eca_agent01_tree")
        # Set the shutdown function (stop the agent)
        rospy.on_shutdown(self.shutdown)
        # Publisher to manually control the agent (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        rate = rospy.Rate(10)

         # initialize existence
        black_board_object.ex = None
        # initialize primitive interactions
        primitive_interactions = {"move forward wall": ("e1", "r1", 50),\
                                  "move forward no wall": ("e1", "r4", -20),\
                                  "move forward fail": ("e1", "r10", -50),\
                                  "turn left": ("e2", "r2", 15),\
                                  "turn right": ("e3", "r3", 25),\
#                                  "front free": ("e4", "r4", 1),\
#                                  "front busy": ("e4", "r5", -2),\
                                  "right1 sensing": ("e5", "r6", 20),\
                                  "right2 sensing": ("e5", "r8", 10),\
                                  "right3 sensing": ("e5", "r12", 10),\
                                  "nothing on right1": ("e5", "r14", -50),\
                                  "left1 sensing": ("e6", "r7", 0),\
                                  "left2 sensing": ("e6", "r9", 0),\
                                  "left3 sensing": ("e6", "r11", 0),\
                                  "nothing on left1": ("e6", "r13", 0)
}
        # initialize environments and existences
        self.mechanism = black_board_object.agent_mechanism
        if self.mechanism == "simple":
            black_board_object.environment = Environment()
            black_board_object.ex = Existence(primitive_interactions, black_board_object.environment)
        elif self.mechanism == "recursive":
            black_board_object.environment = Environment()
            black_board_object.ex = RecursiveExistence(primitive_interactions, black_board_object.environment)
        elif self.mechanism == "constructive":
            black_board_object.environment = ConstructiveEnvironment()
            black_board_object.ex = ConstructiveExistence(primitive_interactions, black_board_object.environment)
        # Create the root node
        ECAAGENT02 = Sequence("ECAAGENT02")
        
        START_STEP = CallbackTask("START STEP", black_board_object.ex.step)
        
        I_F_IS_VISITED =IgnoreFailure("I_F IS VISITED")
        
        IS_VISITED = CallbackTask("is visited", self.is_visited)
        
        ECAAGENT02.add_child(START_STEP)
        ECAAGENT02.add_child(I_F_IS_VISITED)
        
        # Display the tree before beginning execution
        print bcolors.HEADER + "ECAAGENT02 Behavior Tree" + bcolors.ENDC
        print_tree(ECAAGENT02, indent=0, use_symbols=True)
        print_dot_tree(ECAAGENT02, dotfilepath='/home/juan/catkin_ws/src/adaptor001/tree02.dot')
        
        # Run the tree
        while not rospy.is_shutdown():
            #pdb.set_trace()
            ECAAGENT02.run()
            decoded = Decode(black_board_object.step_trace)
            translated = decoded.get_translation()
            print bcolors.OKGREEN + str(black_board_object.sim_step) + " " +  str(translated) + bcolors.ENDC
            print "\n"
            
            #raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
            
            if len(black_board_object.ex.INTERACTIONS) >= self.INTERACTION_ENACTION_HISTORY_SIZE:
                black_board_object.ex.INTERACTIONS.popitem(last=False)
                
            if black_board_object.sim_step >= 3:
                black_board_object.boredom = True
            black_board_object.sim_step += 1
            rate.sleep()   
    
    def is_visited(self):
        if black_board_object.move_count == 0:
            rospy.loginfo("Waypoint is not visited.")
            return True
        if (black_board_object.agent_position, black_board_object.agent_rotation) in black_board_object.waypoints[0:-1]:
            rospy.loginfo("Waypoint is visited.")
            return True
        else:
            rospy.loginfo("Waypoint is not visited.")
            return True
        
    def shutdown(self):
        rospy.loginfo("Stopping the agent...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    

class Existence:
    """
    A class implementing the agent control-and-learning mechanism.
    The agent operates by executing and learning interactions, and is 
    motivated to perform those interactions that have positive valences.
    Interactions can be of two types. Primitive interactions are tuples of 
    (experiment, result, valence). Composite interactions are interactions 
    which consist of primitive interactions.
    When a given experiment is performed and a given result is obtained, 
    the corresponding interaction is considered enacted.
    """
    EXPERIMENTS = OrderedDict()
    INTERACTIONS = OrderedDict()
    RESULTS = OrderedDict()

    def __init__(self, primitive_interactions, environment):
        """
        Initialize existence with a set of primitive interactions and 
        environment.
        :param primitive_interactions: (dict) of primitive interactions 
        of the form {(str) interaction meaning: ((str) experiment, 
        (str) result, (int) valence)}
        :param environment: (Environment) that controls which results are 
        returned for a given primitive experiment
        :return: (Existence)
        """
        self.context_interaction = None
        self.mood = None
        self.environment = environment
        self.primitive_interactions = primitive_interactions
        self.initialize_interactions(primitive_interactions)
        
    def step(self):
        """
        Execute a single simulation step.
        :return: (str) performed interaction and mood
        """
        #pdb.set_trace
        print bcolors.OKGREEN + "Context: " + str(self.context_interaction) + bcolors.ENDC
        anticipations = self.anticipate()  # anticipate possible interactions
        experiment = self.select_experiment(anticipations)  # select the best experiment
        result_label = self.environment.return_result(experiment)  # consult the world and return result
        result = self.addget_result(result_label)  # add result to the dictionary
        enacted_interaction = self.get_interaction(experiment.get_label() + result.get_label())
        print bcolors.OKGREEN + "Enacted " + str(enacted_interaction) + bcolors.ENDC

        if enacted_interaction.get_valence() > 0:
            self.mood = 'HAPPY'
        else:
            self.mood = 'SAD'

        self.learn_composite_interaction(self.context_interaction, enacted_interaction)
        self.context_interaction = enacted_interaction

        black_board_object.step_trace = experiment.get_label() + result.get_label() + " " + self.mood
        
        return 1

    def initialize_interactions(self, primitive_interactions):
        """
        Add primitive interactions to existence
        :param primitive_interactions: a set of primitive interactions 
        provided as a dictionary
        {(str) interaction meaning: ((str) experiment, 
        (str) result, (int) valence)}
        """
        for interaction in primitive_interactions:
            meaning = interaction
            experiment_label = primitive_interactions[interaction][0]
            result_label = primitive_interactions[interaction][1]
            valence = primitive_interactions[interaction][2]
            result = self.addget_result(result_label)
            experiment = self.addget_experiment(experiment_label)
            self.addget_primitive_interaction(experiment, result, valence, meaning)

    def addget_primitive_interaction(self, experiment, result, valence=None, meaning=None):
        """
        If a primitive interaction is not in the INTERACTIONS dictionary, 
        add it. Otherwise just return it.
        :param experiment: (str) primitive experiment
        :param result: (str) primitive result
        :param valence: (int) valence of the interaction
        :param meaning: (str) observer's meaning of the interaction
        :return: (interaction) primitive interaction from the INTERACTIONS dictionary
        """
        label = experiment.get_label() + result.get_label()
        if label not in self.INTERACTIONS:
            interaction = Interaction(label)
            interaction.set_experiment(experiment)
            interaction.set_result(result)
            interaction.set_valence(valence)
            interaction.set_meaning(meaning)
            self.INTERACTIONS[label] = interaction
        return self.INTERACTIONS[label]

    def learn_composite_interaction(self, context_interaction, enacted_interaction):
        """
        Learn a new composite interaction or reinforce it if already known.
        :param context_interaction: (Interaction) at time t-1
        :param enacted_interaction: (Interaction) just performed
        """
        #pdb.set_trace
        if context_interaction is not None:
            label = context_interaction.get_label() + enacted_interaction.get_label()
            if label not in self.INTERACTIONS:
                # valence is a sum of the two valences of both primitive interactions
                valence = context_interaction.get_valence() + enacted_interaction.get_valence()
                interaction = Interaction(label)
                interaction.set_pre_interaction(context_interaction)
                interaction.set_post_interaction(enacted_interaction)
                interaction.set_valence(valence)
                self.INTERACTIONS[label] = interaction
                print bcolors.OKGREEN + "Learn " + label + bcolors.ENDC
            else:
                interaction = self.INTERACTIONS[label]
                print bcolors.OKGREEN + 'Incrementing weight for ' + str(interaction) + bcolors.ENDC
                interaction.increment_weight()

    def anticipate(self):
        """
        Anticipate possible interactions based on current context.
        :return: (list) of Anticipations
        """
        #pdb.set_trace
        anticipations = []
        if self.context_interaction is not None:
            activated_interactions = self.get_activated_interactions()
            for activated_interaction in activated_interactions:
                # retrieve proposed interactions
                proposed_interaction = activated_interaction.get_post_interaction()
                # proclivity is a product of the weight of the whole interaction and a valence of proposed
                proclivity = activated_interaction.get_weight() * proposed_interaction.get_valence()
                anticipations.append(Anticipation(proposed_interaction, proclivity))
                print bcolors.OKGREEN + "Afforded: ", proposed_interaction, " proclivity: " + str(proclivity) + bcolors.ENDC
        return anticipations

    def get_activated_interactions(self):
        """
        Retrieve activated interactions based on current context.
        :return: (list) of Interactions
        """
        activated_interactions = []
        # loop through all known interactions
        for key in self.INTERACTIONS:
            activated_interaction = self.INTERACTIONS[key]
            # see if known interaction's pre-interactions is the same as interaction performed at t-1
            if activated_interaction.get_pre_interaction() == self.context_interaction:
                activated_interactions.append(activated_interaction)
        return activated_interactions

    def select_experiment(self, anticipations):
        """Select experiment from proposed anticipations"""
        #pdb.set_trace
        if len(anticipations) > 0:
            #anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by proclivity
            anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by valence
            afforded_interaction = anticipations[0].get_interaction()
            if afforded_interaction.get_valence() >= 0:
                intended_interaction = afforded_interaction
                print bcolors.OKGREEN + "Intending " + str(intended_interaction) + bcolors.ENDC
                chosen_experiment = intended_interaction.get_experiment()
            else:
                # if proposed interaction leads to negative valence, choose at random
                chosen_experiment = self.get_random_experiment(afforded_interaction)
                print bcolors.OKGREEN + "Don't like the affordance, intending experiment " + chosen_experiment.get_label() + bcolors.ENDC
        else:
            # if nothing was anticipated, choose at random
            # we have decided to use the max valence interaction instead
            valence_max = 0.0
            for interaction in self.primitive_interactions:
                valence = self.primitive_interactions[interaction][2]
                if valence_max < valence:
                    valence_max = valence
                    experiment = self.primitive_interactions[interaction][0]
            chosen_experiment = self.EXPERIMENTS[experiment]
            #chosen_experiment = self.get_random_experiment(None)
            print bcolors.OKGREEN + "Don't know what to do, intending experiment " + chosen_experiment.get_label() + bcolors.ENDC
        return chosen_experiment

    def get_random_experiment(self, interaction):
        random_experiment = random.choice(self.EXPERIMENTS.values())
        # we have decided to use the max valence interaction instead
        if interaction is None:
            valence_max = 0.0
            for interact in self.primitive_interactions:
                valence = self.primitive_interactions[interact][2]
                if valence_max < valence:
                    valence_max = valence
                    experiment = self.primitive_interactions[interact][0]
            best_experiment = self.EXPERIMENTS[experiment]
            return best_experiment
        else:
            # trying to choose a random experiment but avoid choosing one that was part of the rejected interaction
            bad_experiment = interaction.get_experiment()
            chosen_experiment = random.choice(self.EXPERIMENTS.values())
            while chosen_experiment == bad_experiment:
                chosen_experiment = random.choice(self.EXPERIMENTS.values())
            return random_experiment

    def addget_result(self, label):
        if label not in self.RESULTS:
            self.RESULTS[label] = Result(label)
        return self.RESULTS[label]

    def addget_experiment(self, label):
        if label not in self.EXPERIMENTS:
            self.EXPERIMENTS[label] = Experiment(label)
        return self.EXPERIMENTS[label]

    def addget_interaction(self, label):
        if label not in self.INTERACTIONS:
            self.INTERACTIONS[label] = Interaction(label)
        return self.INTERACTIONS[label]

    def get_interaction(self, label):
        if label in self.INTERACTIONS:
            return self.INTERACTIONS[label]
        else:
            return None
        

class RecursiveExistence(Existence):
    """
    Implements recursive self-programming.
    Context is now of depth 2: prev_context_interaction at t-2, 
    and context_interaction at t-1
    """
    def __init__(self, primitive_interactions, environment):
        """
        Initialize existence with a set of primitive interactions provided 
        as a dictionary:
        {(str) interaction meaning: ((str) experiment, (str) result, 
        (int) valence)
        """
        Existence.__init__(self, primitive_interactions, environment)
        self.context_pair_interaction = None  # context at previous two steps (t-2, t-1)

    def step(self):
        
        print bcolors.OKGREEN + "Memory: " + bcolors.ENDC
        # translate the coded way of interactions in its clear meaning
        for i in self.INTERACTIONS:
            decoded = Decode(str(i))
            translated = decoded.get_translation()
            print bcolors.OKGREEN + translated + bcolors.ENDC
        print "\n"
        #raw_input(bcolors.OKGREEN + "Press ENTER to continue..." + bcolors.ENDC)
        
        anticipations = self.anticipate()
        for anticipation in anticipations:
            print bcolors.OKGREEN + "Anticipated: " + str(anticipation) + bcolors.ENDC
            
        pdb.set_trace()
            
        experiment = self.select_experiment(anticipations)  # recursive experiment
        print bcolors.OKGREEN + "Selected experiment: " + experiment.get_label() + bcolors.ENDC
        intended_interaction = experiment.get_intended_interaction()
        print bcolors.OKGREEN + "Intending: " + intended_interaction.__repr__() + bcolors.ENDC
        print bcolors.OKGREEN + "Intending experiment: ", intended_interaction.get_experiment().get_label() + bcolors.ENDC
        enacted_interaction = self.enact(intended_interaction)

        print bcolors.OKGREEN + "Enacted " + str(enacted_interaction) + bcolors.ENDC
        if enacted_interaction != intended_interaction and experiment.is_abstract:
            failed_result = self.addget_result(enacted_interaction.get_label().upper())
            print bcolors.OKGREEN + "failed result: ", failed_result.get_label() + bcolors.ENDC
            valence = enacted_interaction.get_valence()
            print bcolors.OKGREEN + "experiment: ", str(experiment) + bcolors.ENDC
            enacted_interaction = self.addget_primitive_interaction(experiment, failed_result, valence)
            print bcolors.OKGREEN + "Really enacted " + str(enacted_interaction) + bcolors.ENDC

        if enacted_interaction.get_valence() >= 0:
            self.mood = 'HAPPY'
        else:
            self.mood = 'SAD'

        # learn context_pair_interaction, context_interaction, enacted_interaction
        self.learn_recursive_interaction(enacted_interaction)
        black_board_object.step_trace = enacted_interaction.__repr__() + " " + self.mood
        
#        print self.EXPERIMENTS
#        print "\n"
#        print self.INTERACTIONS
#        print "\n"
#        print self.RESULTS
#        print "\n"
#        
#        raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
        
        return 1

    def initialize_interactions(self, primitive_interactions):
        for interaction in primitive_interactions:
            meaning = interaction
            experiment_label = primitive_interactions[interaction][0]
            result_label = primitive_interactions[interaction][1]
            valence = primitive_interactions[interaction][2]
            experiment = self.addget_abstract_experiment(experiment_label)
            result = self.addget_result(result_label)
            self.addget_primitive_interaction(experiment, result, valence, meaning)

        for experiment in self.EXPERIMENTS.values():
            interaction = Interaction(experiment.get_label() + "r2")
            interaction.set_valence(1)
            interaction.set_experiment(experiment)
            experiment.set_intended_interaction(interaction)

    def addget_abstract_experiment(self, label):
        if label not in self.EXPERIMENTS:
            experiment = RecursiveExperiment(label)
            self.EXPERIMENTS[label] = experiment
        return self.EXPERIMENTS[label]

    def enact(self, intended_interaction):
        if intended_interaction.is_primitive():
            return self.enact_primitive_interaction(intended_interaction)
            # experiment = intended_interaction.get_experiment()
            # result = self.return_result(experiment)
            # return experiment, result
        else:
            # enact pre-interaction
            enacted_pre_interaction = self.enact(intended_interaction.get_pre_interaction())
            if enacted_pre_interaction != intended_interaction.get_pre_interaction():
                return enacted_pre_interaction
            else:
                # enact post-interaction
                enacted_post_interaction = self.enact(intended_interaction.get_post_interaction())
                return self.addget_composite_interaction(enacted_pre_interaction, enacted_post_interaction)

    def enact_primitive_interaction(self, intended_interaction):
        """
        Implements the cognitive coupling between the agent and the environment
        Tries to enact primitive intended_interaction.
        """
        experiment = intended_interaction.get_experiment()
        result_label = self.environment.return_result(experiment)
        result = self.addget_result(result_label)
        return self.addget_primitive_interaction(experiment, result)

    def select_experiment(self, anticipations):
        #pdb.set_trace()
        if black_board_object.move_count > 1:
            anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by valence
            selected_anticipation = anticipations[0]
            return selected_anticipation.get_experiment()
        else:
            # if nothing was anticipated, choose at random
            # we have decided to use the max valence interaction instead
            valence_max = 0.0
            for interaction in self.primitive_interactions:
                valence = self.primitive_interactions[interaction][2]
                if valence_max < valence:
                    valence_max = valence
                    experiment = self.primitive_interactions[interaction][0]
            chosen_experiment = self.EXPERIMENTS[experiment]
            return chosen_experiment

    def anticipate(self):
        anticipations = self.get_default_anticipations()
        # print "Default anticipations: ", anticipations
        if self.context_interaction is not None:
            activated_interactions = self.get_activated_interactions()
            for activated_interaction in activated_interactions:
                # print "activated interaction: ", activated_interaction
                experiment = activated_interaction.get_post_interaction().get_experiment()
                # print "activated experiment: " + experiment.get_label()
                proclivity = activated_interaction.get_weight() * activated_interaction.get_post_interaction().get_valence()
                # print "activated proclivity: " + str(proclivity)
                anticipation = RecursiveAnticipation(experiment, proclivity)
                # print "activated anticipation: ", anticipation
                if anticipation not in anticipations:
                    anticipations.append(anticipation)
                else:
                    index = anticipations.index(anticipation)
                    anticipations[index].add_proclivity(proclivity)
                # print "Afforded ", anticipation
        return anticipations

    def get_default_anticipations(self):
        """All known experiments are proposed by default with proclivity 0"""
        anticipations = []
        for experiment in self.EXPERIMENTS.values():
            if not experiment.is_abstract:
                anticipation = RecursiveAnticipation(experiment, 0)
                anticipations.append(anticipation)
        random.shuffle(anticipations) # shuffle order
        return anticipations

    def get_activated_interactions(self):
        context_interactions = []
        if self.context_interaction is not None:
            context_interactions.append(self.context_interaction)
            if not self.context_interaction.is_primitive():
                context_interactions.append(self.context_interaction.get_post_interaction())
            if self.context_pair_interaction is not None:
                context_interactions.append(self.context_pair_interaction)
        print bcolors.OKGREEN + "Context: " + repr(context_interactions) + bcolors.ENDC
        activated_interactions = []
        for key in self.INTERACTIONS:
            activated_interaction = self.INTERACTIONS[key]
            if not activated_interaction.is_primitive():
                if activated_interaction.get_pre_interaction() in context_interactions:
                    activated_interactions.append(activated_interaction)
        for activated_interaction in activated_interactions:
            print bcolors.OKGREEN + "Activated: " + str(activated_interaction) + bcolors.ENDC
        return activated_interactions

    def get_context_interaction(self):
        return self.context_interaction

    def set_context_interaction(self, enacted_interaction):
        self.context_interaction = enacted_interaction

    def get_context_pair_interaction(self):
        return self.context_pair_interaction

    def set_context_pair_interaction(self, enacted_pair_interaction):
        self.context_pair_interaction = enacted_pair_interaction

    # Learning:
    def learn_recursive_interaction(self, enacted_interaction):
        enacted_pair_interaction = None
        if self.context_interaction is not None:
            # if hist[-1]: learn(hist[-1], enacted)
            enacted_pair_interaction = self.addreinforce_composite_interaction(self.context_interaction, enacted_interaction)

            if self.context_pair_interaction is not None:
                # if hist[-1] and hist[-2]
                # learn <penultimate <previous current>>
                self.addreinforce_composite_interaction(self.context_pair_interaction.get_pre_interaction(), enacted_pair_interaction)
                # learn <<penultimate previous> current>
                self.addreinforce_composite_interaction(self.context_pair_interaction, enacted_interaction)

        self.set_context_interaction(enacted_interaction)
        self.set_context_pair_interaction(enacted_pair_interaction)

    def addreinforce_composite_interaction(self, pre_interaction, post_interaction):
        #pdb.set_trace
        composite_interaction = self.addget_composite_interaction(pre_interaction, post_interaction)
        composite_interaction.increment_weight()

        if composite_interaction.get_weight() == 1:
            print bcolors.OKGREEN + "Learned: " + str(composite_interaction) + bcolors.ENDC
        else:
            print bcolors.OKGREEN + "Reinforced: " + str(composite_interaction) + bcolors.ENDC

        return composite_interaction

    def addget_composite_interaction(self, pre_interaction, post_interaction):
        """Record in or get from a composite interaction in memory.
        If a new composite interaction is created, then a new abstract 
        experience is also created and associated to it.
        """
        pdb.set_trace
        label = "<" + pre_interaction.get_label() + post_interaction.get_label() + ">"
        interaction = self.get_interaction(label)
        if interaction is None:
            interaction = self.addget_interaction(label)
            interaction.set_pre_interaction(pre_interaction)
            interaction.set_post_interaction(post_interaction)
            valence = pre_interaction.get_valence() + post_interaction.get_valence()
            interaction.set_valence(valence)
            experiment_label = interaction.get_label().upper()
            new_experiment = self.addget_abstract_experiment(experiment_label)
            new_experiment.set_abstract()
            new_experiment.set_intended_interaction(interaction)
            interaction.set_experiment(new_experiment)
        return interaction
    

class ConstructiveExistence(RecursiveExistence):
    """
    In constructive existence the basic unit of analysis and implementation
    is interaction, not experiments and results.
    """
    def __init__(self, primitive_interactions, environment):
        RecursiveExistence.__init__(self, primitive_interactions, environment)

    # Existence 50.2
    def step(self):
        #pdb.set_trace
        print bcolors.OKGREEN + "Memory: " + bcolors.ENDC
        for i in self.INTERACTIONS:
            decoded = Decode(str(i))
            translated = decoded.get_translation()
            print bcolors.OKGREEN + translated + bcolors.ENDC
        print "\n"
        #raw_input(bcolors.OKGREEN + "Press ENTER to continue..." + bcolors.ENDC)
        
        anticipations = self.anticipate()
        for anticipation in anticipations:
            print bcolors.OKGREEN + "Anticipated: " + str(anticipation) + bcolors.ENDC
        intended_interaction = self.select_interaction(anticipations)
        print bcolors.OKGREEN + "Intended interaction: " + str(intended_interaction) + bcolors.ENDC
        enacted_interaction = self.enact(intended_interaction)
        print bcolors.OKGREEN + "Enacted interaction: " + str(enacted_interaction) + bcolors.ENDC

        # if intended interaction failed, record the alternative
        if enacted_interaction != intended_interaction:
            intended_interaction.add_alternative_interaction(enacted_interaction)
            print bcolors.OKGREEN + "Alternative interactions:" + str(intended_interaction.get_alternative_interactions()) + bcolors.ENDC

        if enacted_interaction.get_valence() >= 0:
            self.mood = 'HAPPY'
        else:
            self.mood = 'SAD'

        self.learn_recursive_interaction(enacted_interaction)
        black_board_object.step_trace = enacted_interaction.__repr__() + " " + self.mood
        
#        print self.EXPERIMENTS
#        print "\n"
#        print self.INTERACTIONS
#        print "\n"
#        print self.RESULTS
#        print "\n"
#        
#        raw_input(bcolors.WARNING + "Press ENTER to continue..." + bcolors.ENDC)
        
        return 1

    def initialize_interactions(self, primitive_interactions):
        for key in primitive_interactions:
            meaning = key
            experiment_label = primitive_interactions[key][0]
            result_label = primitive_interactions[key][1]
            interaction_label = experiment_label + result_label
            valence = primitive_interactions[key][2]
            primitive_interaction = self.addget_interaction(interaction_label)
            primitive_interaction.set_valence(valence)
            primitive_interaction.set_meaning(meaning)
            # creating default experiments to begin with
            self.addget_abstract_experiment(primitive_interaction)

    def addget_abstract_experiment(self, interaction):
        """
        All experiments are now abstract, namely they are interactions.
        """
        #pdb.set_trace
        label = interaction.get_label().upper()
        if label not in self.EXPERIMENTS:
            abstract_experiment = RecursiveExperiment(label)
            abstract_experiment.set_intended_interaction(interaction)
            abstract_experiment.set_abstract()
            interaction.set_experiment(abstract_experiment)
            self.EXPERIMENTS[label] = abstract_experiment
        return self.EXPERIMENTS[label]

    def addget_composite_interaction(self, pre_interaction, post_interaction):
        """
        Record in or get from a composite interaction in memory.
        If a new composite interaction is created, then a new abstract 
        experience is also created and associated to it.
        """
        #pdb.set_trace
        label = "<" + pre_interaction.get_label() + post_interaction.get_label() + ">"
        interaction = self.get_interaction(label)
        if interaction is None:
            interaction = self.addget_interaction(label)
            interaction.set_pre_interaction(pre_interaction)
            interaction.set_post_interaction(post_interaction)
            valence = pre_interaction.get_valence() + post_interaction.get_valence()
            interaction.set_valence(valence)
            self.addget_abstract_experiment(interaction)
        return interaction

    # Existence 50.2
    def anticipate(self):
        #pdb.set_trace
        anticipations = self.get_default_anticipations()
        print bcolors.OKGREEN + "Default anticipations: " + str(anticipations) + bcolors.ENDC
        activated_interactions = self.get_activated_interactions()
        # print "Activated interactions: ", activated_interactions
        if self.context_interaction is not None:
            for activated_interaction in activated_interactions:
                proposed_interaction = activated_interaction.get_post_interaction()
                # print "activated experiment: " + experiment.get_label()
                proclivity = activated_interaction.get_weight() * proposed_interaction.get_valence()
                anticipation = ConstructiveAnticipation(proposed_interaction, proclivity)
                # print "activated anticipation: " + anticipation.__repr__()
                if anticipation not in anticipations:
                    anticipations.append(anticipation)
                else:
                    index = anticipations.index(anticipation)
                    # increment proclivity if anticipation is already in the list
                    anticipations[index].add_proclivity(proclivity)
                # print "Afforded " + anticipation.__repr__()

            for anticipation in anticipations:
                index = anticipations.index(anticipation)
                alternative_interactions = anticipation.get_interaction().get_alternative_interactions()
                for interaction in alternative_interactions:
                    for activated_interaction in activated_interactions:
                        # combine proclivity with alternative interactions
                        if interaction == activated_interaction.get_post_interaction():
                            proclivity = activated_interaction.get_weight() * interaction.get_valence()
                            anticipations[index].add_proclivity(proclivity)
        return anticipations

    # Existence 50.2
    def get_default_anticipations(self):
        anticipations = []
        for interaction in self.INTERACTIONS.values():
            if interaction.is_primitive():
                # print "interaction is primitive"
                anticipation = ConstructiveAnticipation(interaction, 0)
                # print "adding anticipation", anticipation
                anticipations.append(anticipation)
        # sort default anticipations by valence - this could be random...
        anticipations.sort(key=lambda x: x.get_interaction().get_valence(), reverse=True)
        #anticipations.sort(key=lambda x: x.get_interaction().get_label())
        return anticipations

    def enact(self, intended_interaction):
        #pdb.set_trace
        # if interaction is primivite, consult the world and get what was actually enacted
        if intended_interaction.is_primitive():
            enacted_interaction_label = self.environment.enact_primitive_interaction(intended_interaction)
            enacted_interaction = self.addget_interaction(enacted_interaction_label)
            return enacted_interaction
        else:
            # if interaction is composite, try to enact its pre-interaction
            enacted_pre_interaction = self.enact(intended_interaction.get_pre_interaction())
            # if enacting failed, break the sequence and return
            if enacted_pre_interaction != intended_interaction.get_pre_interaction():
                return enacted_pre_interaction
            else:
                # if enacting pre-interaction succeeded, try to enact post-interaction
                enacted_post_interaction = self.enact(intended_interaction.get_post_interaction())
                return self.addget_composite_interaction(enacted_pre_interaction, enacted_post_interaction)

    def select_interaction(self, anticipations):
        anticipations.sort(key=lambda x: x.compare(), reverse=True)  # choose by proclivity
        selected_anticipation = anticipations[0]
        intended_interaction = selected_anticipation.get_interaction()
        # if intended_interaction.get_valence() < 0:
        #     intended_interaction = self.get_random_interaction(intended_interaction)
        #     print "Don't like the affordance, intending random interaction..."
        return intended_interaction

    def get_random_interaction(self, interaction):
        random_interaction = random.choice(self.INTERACTIONS.values())
        if interaction is None:
            return random_interaction
        else:
            bad_experiment = interaction.get_experiment()
            chosen_experiment = random_interaction.get_experiment()
            #while chosen_experiment == bad_experiment:
            while chosen_experiment != 'e1':    
                random_interaction = random.choice(self.INTERACTIONS.values())
            return random_interaction


# the kind of boredom handler that is going to be used
boredom_handler = RepetitiveBoredomHandler()
    
if __name__ == '__main__':
    #pdb.set_trace()
    # run with  i.e. rosrun eca_agent01.py constructive
    parser = argparse.ArgumentParser()
    parser.add_argument("mechanism", type=str, help="specify the learning mechanism to be used",
                        choices=["simple", "recursive", "constructive"])
    args = parser.parse_args()
    
    black_board_object.agent_mechanism = args.mechanism
    
    tree = EcaAgent02()
  