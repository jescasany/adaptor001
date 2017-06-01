#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed May 10 11:51:37 2017

@author: juan
"""

import abc
from collections import Counter
import math
from fancy_prompts import bcolors
import black_board
from black_board_class import bbo

# class that represent an agent's boredom handler.    
class BoredomHandler(object):
    """
    Abstract boredom handler class.
    """
    @abc.abstractmethod
    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        """
        Modifies the valence of an interaction such that boredom is handled.

        :param INTERACTIONS: The interaction memory
        :param interaction: The interaction to process boredom for
        :param unmodified_valence: The unmodified (raw) valence of the interaction
        :return: The modified valence taking boredom into account
        """
        raise NotImplementedError("Should be implemented by child")


class PassthroughBoredomHandler(BoredomHandler):
    """
    A boredom handler not implementing any boredom measures.
    """
    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        return unmodified_valence

class WeightBoredomHandler(BoredomHandler):
    """
    A boredom handler taking into account the weight of interactions. The sum
    of the hierarchical weight of an interaction is calculated, and its
    contribution to the total weight is calculated. This is used to discount
    interactions that have a high contribution.
    """
    def interaction_total_weight(self, INTERACTIONS, interaction):
        """
        Get the total (hierarchical) weight of an interaction. This takes the
        sum of all weights of all interactions inside the hierarchy of this
        interaction. E.g., for a composite interaction <i1, i2> the sum is
        weight(<i1, i2>) = <i1, i2>.weight + weight(i1) + weight(i2).

        :param INTERACTIONS: The interaction memory
        :param interaction: The interaction to get the hierarchical weight for
        :return: The hierarchical weight of the interaction
        """
        if interaction.is_primitive():
            return interaction.get_weight()
        else:
            return (
                interaction.get_weight() 
                + self.interaction_total_weight(INTERACTIONS, interaction.get_pre_interaction()) 
                + self.interaction_total_weight(INTERACTIONS, interaction.get_post_interaction())
            )
    
    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        if unmodified_valence > 0:
            
            sum = 0
            for key in INTERACTIONS:
                sum += INTERACTIONS[key].get_weight()
            weight = self.interaction_total_weight(INTERACTIONS, interaction)
            modifier = (1 - float(weight)/float(sum))
            return unmodified_valence * modifier
        else:
            return unmodified_valence

class RepetitiveBoredomHandler(BoredomHandler):
    """
    A boredom handler taking into the account the last few (primitive)
    interactions enacted by the agent, and compares the similarity of those
    with the proposed interaction. The more similar, the more the interaction
    is penalized.
    """
    
    HISTORY_CONSIDER_SIZE = 15

    def count_interactions(self, interaction_sequence):
        """
        Count the interaction occurrences in a sequence.
        :param interaction_sequence: The interaction sequence
        :return: A Counter (dictionary) object mapping from interactions to
                 their frequency in the sequence.
        """
        count = Counter()
        for interaction_ in interaction_sequence:
            count[interaction_] += 1

        return count

    def similarity(self, count1, count2):
        """
        Calculate the cosine similarity between two counts (Counter dictionaries, seen as vectors).
        :param count1: The first interaction count
        :param count2: The second interaction count
        :return: The cosine similarity between the two counts
        """
        c1_dot_c2 = 0
        c1_len_squared = 0
        c2_len_squared = 0

        for interaction_name in count1:
            c1_dot_c2 += count1[interaction_name] * count2[interaction_name]
            c1_len_squared += count1[interaction_name]**2

        for interaction_name in count2:
            c2_len_squared += count2[interaction_name]**2

        if c1_len_squared == 0:
            return -1
        else:
            return c1_dot_c2 / (math.sqrt(c1_len_squared) * math.sqrt(c2_len_squared))

    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        #pdb.set_trace()
        history = INTERACTIONS.keys()[-self.HISTORY_CONSIDER_SIZE:]
        history_count = self.count_interactions(history)
        interaction_count = self.count_interactions(interaction.unwrap())

        similarity = self.similarity(history_count, interaction_count)
        modifier = 1 - similarity

        return unmodified_valence * modifier

class WeightRepetitiveBoredomHandler(BoredomHandler):
    """
    A boredom handler combining the weight boredom handler and repetitive
    boredom handler by taking the average valence output of the two.
    """
    def __init__(self):
        self.weightBoredomHandler = WeightBoredomHandler()
        self.repetitiveBoredomHandler = RepetitiveBoredomHandler()

    def process_boredom(self, INTERACTIONS, interaction, unmodified_valence):
        modified_valence = (
            self.weightBoredomHandler.process_boredom(INTERACTIONS, interaction, unmodified_valence)
            +
            self.repetitiveBoredomHandler.process_boredom(INTERACTIONS, interaction, unmodified_valence)
            )/2
        print bcolors.OKGREEN + "Interaction modified: " + repr(interaction) + "Modified valence: " + str(modified_valence) + bcolors.ENDC
        return modified_valence
    
# the kind of boredom handler that is going to be used
boredom_handler = RepetitiveBoredomHandler()