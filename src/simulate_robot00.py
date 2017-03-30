from architecture.existence_robot00 import Existence, RecursiveExistence, ConstructiveExistence
from environment_robot00 import TestEnvironmentD1, TestEnvironmentD2, TestEnvironment, Environment, ConstructiveEnvironment
import os
import argparse

__author__ = 'juan'

def main(mechanism):
    """
    The main script that runs the simulation.
    :param mechanism: which mechanism will be used to simulate behavior 
    (simple, recursive, constructive)
    """

    # initialize existence
    ex = None

    # initialize primitive interactions
    primitive_interactions = {"move forward": ("e1", "r1", 5),\
                              "turn left": ("e2", "r2", -1), \
                              "turn right": ("e3", "r3", 1),\
                              "front free": ("e4", "r4", 5),\
                              "front busy": ("e4", "r5", -10),\
                              "right sensing": ("e5", "r6", 5),\
                              "left sensing": ("e6", "r7", 2)}
    
    # initialize environments and existences
    if mechanism == "simple":
        environment = Environment()
        ex = Existence(primitive_interactions, environment)
    elif mechanism == "recursive":
        environment = Environment()
        ex = RecursiveExistence(primitive_interactions, environment)
    elif mechanism == "constructive":
        environment = ConstructiveEnvironment()
        ex = ConstructiveExistence(primitive_interactions, environment)

    i = 1
    # perform one simulation step (that might consist of several primitive steps)
    step_trace = ex.step()
    print (i, step_trace)
    print "\n"
    i += 1

if __name__ == '__main__':
    # run with  python simulate_robot00.py constructive
    parser = argparse.ArgumentParser()
    parser.add_argument("mechanism", type=str, help="specify the learning mechanism to be used",
                        choices=["simple", "recursive", "constructive"])
    args = parser.parse_args()
    main(args.mechanism)

