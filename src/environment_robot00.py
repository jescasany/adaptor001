from eca_robot00 import *
from math import pi

class Environment:
    """
    Class that implements the basic real-world environment.
    """
    def __init__(self):
        self.last_result = None
        self.robot = EcaRobot00()

    def return_result(self, experiment):
        """
        Consult the world and return primitive result in response to 
        the experiment initiated by the agent.
        :param experiment: (Experiment) experiment issued by the agent
        :return: (str) result
        """
        result = None
        self.robot.laser_scan()
        if experiment.get_label() == 'e1':
            black_board.adv_distance = 1.0
            black_board.adv_angle = 0.0
            self.robot.move_adv()
            result = 'r1'  # moved forward
        elif experiment.get_label() == 'e2':
            black_board.adv_distance = 0.0
            black_board.adv_angle = pi/2
            self.robot.move_adv()
            result = 'r2'   # turn left
        elif experiment.get_label() == 'e3':
            black_board.adv_distance = 0.0
            black_board.adv_angle = -pi/2
            self.robot.move_adv()
            result = 'r3'   # turn right
        elif experiment.get_label() == 'e4':
            if black_board.driving_forward:
                result = 'r4'  # front free
            else:
                result = 'r5'  # front busy
        elif experiment.get_label() == 'e5':
            self.robot.right_status()
            if black_board.Right:
                result = 'r6'   # right sensing
            else:
                result = 'r8'   # nothing on the right
        elif experiment.get_label() == 'e6':
            self.robot.left_status()
            if black_board.Right:
                result = 'r7'   # left sensing
            else:
                result = 'r9'   # nothing on the left

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
        experiment = intended_interaction.get_label()[:2]
        result = None
        self.robot.laser_scan()
        if experiment.get_label() == 'e1':
            black_board.adv_distance = 1.0
            black_board.adv_angle = 0.0
            self.robot.move_adv()
            result = 'r1'  # moved forward
        elif experiment.get_label() == 'e2':
            black_board.adv_distance = 0.0
            black_board.adv_angle = pi/2
            self.robot.move_adv()
            result = 'r2'   # turn left
        elif experiment.get_label() == 'e3':
            black_board.adv_distance = 0.0
            black_board.adv_angle = -pi/2
            self.robot.move_adv()
            result = 'r3'   # turn right
        elif experiment.get_label() == 'e4':
            if black_board.driving_forward:
                result = 'r4'  # front free
            else:
                result = 'r5'  # front busy
        elif experiment.get_label() == 'e5':
            self.robot.right_status()
            if black_board.Right:
                result = 'r6'   # right sensing
            else:
                result = 'r8'   # nothing on the right
        elif experiment.get_label() == 'e6':
            self.robot.left_status()
            if black_board.Right:
                result = 'r7'   # left sensing
            else:
                result = 'r9'   # nothing on the left

        enacted_interaction = experiment+result
        self.last_interaction = enacted_interaction

        return enacted_interaction

class TestEnvironmentD1:
    """
    Command-line environment of depth 1.
    Returns r2 when current experiment is different from previous 
    experiment. Returns r1 otherwise.
    """
    def __init__(self):
        self.previous_experiment = None

    def set_previous_experiment(self, previous_experiment):
        self.previous_experiment = previous_experiment

    def get_previous_experiment(self):
        return self.previous_experiment

    def return_result(self, experiment):
        previous_experiment = self.get_previous_experiment()
        current_experiment = experiment.get_label()

        if experiment == previous_experiment:
            result = "r1"
        else:
            result = "r2"

        self.set_previous_experiment(experiment)

        return result


class TestEnvironmentD2:
    """
    Command-line environment of depth 2.
    Returns r2 when current experience equals previous and differs from 
    penultimate. Returns r1 otherwise.
    """
    def __init__(self):
        self.penultimate_experiment = None
        self.previous_experiment = None

    def set_penultimate_experiment(self, penultimate_experiment):
        self.penultimate_experiment = penultimate_experiment

    def get_penultimate_experiment(self):
        return self.penultimate_experiment

    def set_previous_experiment(self, previous_experiment):
        self.previous_experiment = previous_experiment

    def get_previous_experiment(self):
        return self.previous_experiment

    def return_result(self, experiment):
        penultimate_experiment = self.get_penultimate_experiment()
        previous_experiment = self.get_previous_experiment()
        current_experiment = experiment.get_label()

        if experiment == previous_experiment and \
            experiment != penultimate_experiment:
            result = "r2"
        else:
            result = "r1"

        self.set_penultimate_experiment(previous_experiment)
        self.set_previous_experiment(experiment)

        return result

class TestEnvironment:
    """
    Command-line environment of depth 2, which implements 
    constructive principle.
    Returns r2 when current experience equals previous and differs 
    from penultimate.
    Returns R1 otherwise.
    """
    def __init__(self):
        self.penultimate_interaction = None
        self.previous_interaction = None

    def set_penultimate_interaction(self, penultimate_interaction):
        self.penultimate_interaction = penultimate_interaction

    def get_penultimate_interaction(self):
        return self.penultimate_interaction

    def set_previous_interaction(self, previous_interaction):
        self.previous_interaction = previous_interaction

    def get_previous_interaction(self):
        return self.previous_interaction

    def enact_primitive_interaction(self, intended_interaction):
        penultimate_interaction = self.get_penultimate_interaction()
        # print "penultimate interaction", penultimate_interaction
        previous_interaction = self.get_previous_interaction()
        # print "previous interaction", previous_interaction

        if "e1" in intended_interaction.get_label():
            if previous_interaction is not None \
                    and (penultimate_interaction is None or \
                         "e2" in penultimate_interaction and \
                         "e1" in previous_interaction):
                enacted_interaction = "e1r2"
            else:
                enacted_interaction = "e1r1"
        else:
            if previous_interaction is not None \
                    and (penultimate_interaction is None or \
                         "e1" in penultimate_interaction and \
                         "e2" in previous_interaction):
                enacted_interaction = "e2r2"
            else:
                enacted_interaction = "e2r1"

        self.set_penultimate_interaction(previous_interaction)
        self.set_previous_interaction(enacted_interaction)

        return enacted_interaction
