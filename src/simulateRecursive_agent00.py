from architecture.existence import RecursiveExistence
from environment import Environment
import os
import sys
import argparse

__author__ = 'juan'


def main(saveimg):

    wd = os.getcwd()

    primitive_interactions = {"move forward": ("e1", "r1", 5),\
                              "turn left": ("e2", "r2", -1), \
                              "turn right": ("e3", "r3", 1),\
                              "front free": ("e4", "r4", 5),\
                              "front busy": ("e4", "r5", -10),\
                              "right sensing": ("e5", "r6", 5),\
                              "left sensing": ("e6", "r7", 2)}
    
    ex = RecursiveExistence(primitive_interactions, environment)
    i = 1

    anticipations = ex.anticipate()
    experiment = ex.select_experiment(anticipations)

    intended_interaction = experiment.get_intended_interaction()
    intended_interaction.set_experiment(experiment)
    enacted_interaction = ex.enact(intended_interaction)

    if enacted_interaction != intended_interaction and experiment.is_abstract:
        failed_result = ex.addget_result(enacted_interaction.get_label().upper())
        valence = enacted_interaction.get_valence()
        enacted_interaction = ex.addget_primitive_interaction(experiment, failed_result, valence)

    if enacted_interaction.get_valence() >= 0:
        ex.mood = 'HAPPY'
    else:
        ex.mood = 'SAD'

    # learn context_pair_interaction, context_interaction, enacted_interaction
    ex.learn_recursive_interaction(enacted_interaction)

    print (i, experiment.get_label(), environment.last_result, str(enacted_interaction.get_valence()))

    i += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # parser.add_argument("mechanism", type=str, help="specify the learning mechanism to be used",
    #                     choices=["sequence_learning", "hierarchical_learning"])
    #parser.add_argument("saveimg", help="when set to 1, simulation is saved as images in output folder", type=int)
    parser.add_argument("-s", "--saveimg", help="when specified, simulation is saved as images in output folder",
                        action="store_true")
    args = parser.parse_args()
    main(args.saveimg)
