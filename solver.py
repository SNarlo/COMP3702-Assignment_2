import sys

from problem_spec import ProblemSpec
from robot_config import write_robot_config_list_to_file
from tester import *
from visualiser import Visualiser
import random
import matplotlib.pyplot as pyplot
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
        # rand_x = round(random.random(), 2)
        # rand_y = round(random.random(), 2)
        rand_angles = self.generate_random_angles()
        rand_lengths = self.generate_random_lengths()

        random_config = make_robot_config_from_ee1(self.config.points[0][0], self.config.points[0][1], rand_angles,
                                                   rand_lengths, self.config.ee1_grappled, self.config.ee2_grappled)

        test = test_obstacle_collision(random_config, self.spec, self.obstacles)

        if test:
            return random_config
        else:
            return self.generate_sample()

    def plus_or_minus(self, value, other_value):

        plus_minus = random.randint(0, 1)

        if plus_minus == 0:
            return value + other_value
        else:
            return value - other_value
        
    def generate_intermediate_sample(self, config):

        original_x = config.points[0][0]
        original_y = config.points[0][1]
        original_angles = config.ee1_angles
        original_lengths = config.lengths
        original_ee1_grappled = config.ee1_grappled
        original_ee2_grappled = config.ee2_grappled

        primitive = self.spec.PRIMITIVE_STEP
        new_angles = []
        for i in range(len(original_angles)):
            new_angles.append(self.plus_or_minus(original_angles[i], primitive))
        tuple(new_angles)

        new_config = make_robot_config_from_ee1(original_x, original_y, new_angles, original_lengths,
                                                original_ee1_grappled, original_ee2_grappled)

        return new_config

    def interpolate_path(self, config1, config2):

        successful_nodes = [config1]

        i = 0
        while successful_nodes[i].points != config2.points:
            new = self.generate_intermediate_sample(successful_nodes[i])
            test = test_config_distance(successful_nodes[i], new, self.spec)
            print(new)
            if test:
                successful_nodes.append(new)
                i += 1
        return successful_nodes


    def graph(self, n, k):
        """"
        :param n: number of random sample points
        :param k: number of closest neigbour points
        :return: a roadmap G(V, E) where V is a spec and E, a config
        """

        start_node = GraphNode(self.spec, self.spec.initial)
        goal_node = GraphNode(self.spec, self.spec.goal)
        state_graph = []





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
    input_file = "testcases/3g1_m0.txt"
    output_file = "testcases/example_output.txt"
    spec = ProblemSpec(input_file)

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    steps = []

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of RobotConfig objects such that all configurations are collision free, the
    # distance between 2 successive configurations is less than 1 primitive step, the first configuration is the initial
    # state and the last configuration is the goal state.
    #
    #

    if len(arglist) > 1:
        write_robot_config_list_to_file(output_file, steps)

    g = GraphNode(spec, spec.goal)

    c1 = spec.initial
    c2 = spec.goal

    a = g.interpolate_path(c1, c2)
    print(a)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    # v = Visualiser(spec, steps)


if __name__ == '__main__':
    main(sys.argv[1:])
