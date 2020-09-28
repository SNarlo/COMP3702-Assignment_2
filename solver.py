import sys

from problem_spec import ProblemSpec
from robot_config import write_robot_config_list_to_file
from tester import *
from visualiser import Visualiser
import random
import numpy as np
"""
Template file for you to implement your solution to Assignment 2. Contains a class you can use to represent graph nodes,
and a method for finding a path in a graph made up of GraphNode objects.

COMP3702 2020 Assignment 2 Support Code
"""


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []
        self.obstacles = spec.obstacles

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors

    @staticmethod
    def add_connection(n1, n2):
        """
        Creates a neighbor connection between the 2 given GraphNode objects.

        :param n1: a GraphNode object
        :param n2: a GraphNode object
        """
        n1.neighbors.append(n2)
        n2.neighbors.append(n1)

    def generate_random_lengths(self):
        """
        A method which returns random lengths for each segment
        :return: A random segment length
        """
        random_lengths = []
        amount = self.spec.num_segments
        while amount > 0:
            random_lengths.append((random.uniform(self.spec.min_lengths[0], self.spec.max_lengths[1])))
            amount -= 1
        return tuple(random_lengths)

    def generate_random_angles(self):
        """
        A method which returns a tuple of random angles in radians
        :return: A tuple of random angles for each segment
        """
        random_angles = []
        amount = self.spec.num_segments
        while amount > 0:
            random_angles.append(Angle(random.uniform(0, 2 * math.pi)))
            amount -= 1
        return tuple(random_angles)

    def generate_sample(self):
        """
        Generating a random robot configuration which is collision free
        :return: A robot configuration which is collision free
        """

        rand_angles = self.generate_random_angles()
        rand_lengths = self.generate_random_lengths()

        random_config = make_robot_config_from_ee1(self.config.points[0][0], self.config.points[0][1], rand_angles,
                                                   rand_lengths, self.config.ee1_grappled, self.config.ee2_grappled)

        test = test_obstacle_collision(random_config, self.spec, self.obstacles)

        if test and test_self_collision(random_config, self.spec) and test_environment_bounds(random_config):
            return random_config
        else:
            return self.generate_sample()


    def dist_between(self, config1, config2):
        """
        A method which calculate the Euclidean distance between two configs segments
        :param config1: The start config.
        :param config2: The goal config.
        :return: Euclidean distance of all segments summed.
        """
        dx = 0
        dy = 0
        for a in range(len(config1.points)):
            dx += config1.points[a][0] - config2.points[a][0]
            dy += config1.points[a][1] - config2.points[a][1]

        euclidean_dist = math.sqrt(dx**2 + dy**2)
        return euclidean_dist

    def interpolate_path(self, path):
        """
        A method which produces intermediary samples between the start and goal config,
        each intermediate config < 1 primitive step from the next and last.
        :param path: the list containing the two configurations.
        :return: A list of configurations from the start to the goal.
        """

        steps = []
        for i in range(len(path) - 1):
            config1 = path[i]
            config2 = path[i + 1]

            if config1.ee1_grappled and config2.ee1_grappled and \
                    point_is_close(config1.points[0][0], config1.points[0][1], config2.points[0][0], config2.points[0][1], self.spec.TOLERANCE):
                ee1_grappled = True
                ee2_grappled = False
                x1, y1 = config1.points[0]
                base_angles = config1.ee1_angles
                d_angles = [config2.ee1_angles[i].in_radians() - config1.ee1_angles[i].in_radians() for i in range(self.spec.num_segments)]
                make_config = make_robot_config_from_ee1
            else:
                raise Exception("Invalid configs given.")

            d_lengths = [config2.lengths[i] - config1.lengths[i] for i in range(self.spec.num_segments)]
            num_steps = max(math.ceil(max([abs(da) for da in d_angles]) / self.spec.PRIMITIVE_STEP),
                          math.ceil(max([abs(dl) for dl in d_lengths]) / self.spec.PRIMITIVE_STEP)) + 1
            delta_angles = [d_angles[i] / num_steps for i in range(self.spec.num_segments)]
            delta_lengths = [d_lengths[i] / num_steps for i in range(self.spec.num_segments)]

            for i in range(num_steps):
                angles = [base_angles[j] + (delta_angles[j] * (i + 1)) for j in range(self.spec.num_segments)]
                lengths = [config1.lengths[j] + (delta_lengths[j] * (i + 1)) for j in range(self.spec.num_segments)]
                c = make_config(x1, y1, angles, lengths, ee1_grappled, ee2_grappled)
                steps.append(c)

        return steps

    
    def path_check(self, config1, config2):
        """
        Return true for a valid path, false otherwise.
        :param config1: Configuration 1
        :param config2: Configuration 2
        :return: True or False
        """
        if config1.ee1_grappled and config2.ee1_grappled and \
                point_is_close(config1.points[0][0], config1.points[0][1], config2.points[0][0], config2.points[0][1],
                               self.spec.TOLERANCE):
            ee1_grappled = True
            ee2_grappled = False
            x1, y1 = config1.points[0]
            base_angles = config1.ee1_angles
            d_angles = [config2.ee1_angles[i].in_radians() - config1.ee1_angles[i].in_radians() for i in
                        range(self.spec.num_segments)]
            make_config = make_robot_config_from_ee1
        else:
            raise Exception("Invalid configs given.")

        d_lengths = [config2.lengths[i] - config1.lengths[i] for i in range(self.spec.num_segments)]
        num_steps = max(math.ceil(max([abs(da) for da in d_angles]) / self.spec.PRIMITIVE_STEP),
                      math.ceil(max([abs(dl) for dl in d_lengths]) / self.spec.PRIMITIVE_STEP)) + 1
        delta_angles = [d_angles[i] / num_steps for i in range(self.spec.num_segments)]
        delta_lengths = [d_lengths[i] / num_steps for i in range(self.spec.num_segments)]

        for i in range(num_steps):
            angles = [base_angles[j] + (delta_angles[j] * (i + 1)) for j in range(self.spec.num_segments)]
            lengths = [config1.lengths[j] + (delta_lengths[j] * (i + 1)) for j in range(self.spec.num_segments)]
            config = make_config(x1, y1, angles, lengths, ee1_grappled, ee2_grappled)
            
            if not test_environment_bounds(config):
                return False
            if not test_angle_constraints(config, self.spec):
                return False
            if not test_length_constraints(config, self.spec):
                return False
            if not test_self_collision(config, self.spec):
                return False
            if not test_obstacle_collision(config, self.spec, self.obstacles):
                return False
            
        return True

    def PRM(self, initial, goal):
        """
        A PRM algorithm for sampling and connecting successful strategies in order
        to traverse obstacles.
        :param initial: The initial node.
        :param goal: The goal node.
        :return: A list of successful robot configurations.
        """
        dist_limit = 0.35
        if self.dist_between(self.spec.initial, self.spec.goal) < dist_limit:
            if self.path_check(self.spec.initial, self.spec.goal):
                return [self.spec.initial, self.spec.goal] # if the dist between start and end < dist_limit return

        nodes = [initial, goal]
        while True:
            search_range = 100
            for i in range(search_range):
                print(i)
                try:
                    random_config = self.generate_sample()
                    node = GraphNode(self.spec, random_config)
                    for j in nodes:
                        if self.dist_between(node.config, j.config) < dist_limit:
                            if self.path_check(node.config, j.config):
                                self.add_connection(node, j)
                    nodes.append(node)
                except Exception:
                    "Failed"
            step_list = self.interpolate_path(find_graph_path(self.spec, initial))
            step_list.insert(0, initial.config)
            return step_list


def find_graph_path(spec, init_node):
    """
    This method performs a breadth first search of the state graph and return a list of configs which form a path
    through the state graph between the initial and the goal. Note that this path will not satisfy the primitive step
    requirement - you will need to interpolate between the configs in the returned list.

    You may use this method in your solver if you wish, or can implement your own graph search algorithm to improve
    performance.

    :param spec: ProblemSpec object
    :param init_node: GraphNode object for the initial configuration
    :return: List of configs forming a path through the graph from initial to goal
    """
    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]

    return None


def main(arglist):
    # input_file = arglist[0]
    # output_file = arglist[1]
    input_file = "testcases/3g1_m2.txt"
    output_file = "testcases/output.txt"
    spec = ProblemSpec(input_file)

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    g = GraphNode(spec, spec.goal)
    # path_plan = []
    #
    # for i in range(200):
    #     c = g.generate_sample()
    #     path_plan.append(c)

    steps = []

    path_plan = g.PRM(init_node, goal_node)
    # write_robot_config_list_to_file(output_file, path_plan)

    # Code for your main method can go here.
    #
    # Your code should find a sequence of RobotConfig objects such that all configurations are collision free, the
    # distance between 2 successive configurations is less than 1 primitive step, the first configuration is the initial
    # state and the last configuration is the goal state.
    #
    #

    # if len(arglist) > 1:


    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    v = Visualiser(spec, path_plan)


if __name__ == '__main__':
    main(sys.argv[1:])
