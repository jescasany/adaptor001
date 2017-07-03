#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 11:06:28 2017

@author: juan
"""

import pdb

from black_board_class import bbo

from boredom import boredom_handler


class Interaction:
    """
    An interaction is a basic sensorimotor pattern available to the agent.
    An interaction can be primitive or composite. If primitive, it is an association of experiment and result.
    If composite, it has pre- and post-interaction parts.
    Each interaction has valence and weight.
    """
    def __init__(self, label):
        self.label = label
        self.valence = 0
        self.experiment = None
        self.result = None
        self.meaning = None
        self.weight = 0
        self.pre_interaction = None
        self.post_interaction = None
        self.alternative_interactions = []

    def get_label(self):
        return self.label

    def get_experiment(self):
        return self.experiment

    def set_experiment(self, experiment):
        self.experiment = experiment

    def get_result(self):
        return self.result

    def set_result(self, result):
        self.result = result

    def get_valence(self):
        interaction = bbo.interaction
        if bbo.boredom:
            if self.is_primitive():
                bbo.interaction = None
                return boredom_handler.process_boredom(bbo.ex.INTERACTIONS, interaction, self.valence)
            else:
                pre = self.get_pre_interaction()
                post = self.get_post_interaction()
                bbo.interaction = pre
                valence1 = pre.get_valence()
                bbo.interaction = post
                valence2 = post.get_valence()
                self.valence = valence1 + valence2
                interaction = bbo.interaction1
                bbo.interaction = None
                return boredom_handler.process_boredom(bbo.ex.INTERACTIONS, interaction, self.valence)
        else:
            if self.is_primitive():
                bbo.interaction = None
                return self.valence
            else:
                pre = self.get_pre_interaction()
                post = self.get_post_interaction()
                bbo.interaction = pre
                valence1 = pre.get_valence()
                bbo.interaction = post
                valence2 = post.get_valence()
                self.valence = valence1 + valence2
                bbo.interaction = None
                return self.valence

    def set_valence(self, valence):
        self.valence = valence

    def get_meaning(self):
        return self.meaning

    def set_meaning(self, meaning):
        self.meaning = meaning

    def get_pre_interaction(self):
        return self.pre_interaction

    def set_pre_interaction(self, pre_interaction):
        self.pre_interaction = pre_interaction

    def get_post_interaction(self):
        return self.post_interaction

    def set_post_interaction(self, post_interaction):
        self.post_interaction = post_interaction

    def is_primitive(self):
        return self.pre_interaction is None

    def get_weight(self):
        return self.weight

    def increment_weight(self):
        self.weight += 1

    def add_alternative_interaction(self, interaction):
        if interaction not in self.alternative_interactions:
            self.alternative_interactions.append(interaction)

    def get_alternative_interactions(self):
        return self.alternative_interactions
    
    def unwrap(self):
        if self.is_primitive():
            return[self]
        else:
            """
            Unwrap the composite interaction.
            :return: A list of primitive interactions.
            """
            return self.get_pre_interaction().unwrap() + self.get_post_interaction().unwrap()

    def __repr__(self):
        return "{0}, valence {1}, weight {2}".format(self.get_label(), self.get_valence(), self.get_weight())
       