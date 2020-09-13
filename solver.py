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


    def dist_between(self, node, other_node):

        distance = abs(node[0] - other_node[0])
        distance += abs(node[1] - other_node[1])

        return distance

    def generate_sample(self):
        """
        Generating a random robot configuration
        :return: A robot configuration which is collision free
        """

        rand_x = round(random.random(), 2)
        rand_y = round(random.random(), 2)
        rand_angles = Angle(random.uniform(-180, 180)), Angle(random.uniform(-180, 180)), Angle(
            random.uniform(-180, 180))
        # rand_lengths = (random.uniform(0, 1)), (random.uniform(0, 1)), (random.uniform(0, 1))

        random_config = make_robot_config_from_ee1(rand_x, rand_y, rand_angles, self.config.lengths, False, False)
        test = test_obstacle_collision(random_config, self.spec, self.obstacles)

        if test:
            return random_config
        else:
            return self.generate_sample()


    def graph(self, n, k):
        """

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
    input_file = "testcases/3g1_m1.txt"
    output_file = "example_output.txt"
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
    
    a = g.generate_sample()
    print(a)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    # v = Visualiser(spec, steps)

if __name__ == '__main__':
    main(sys.argv[1:])
