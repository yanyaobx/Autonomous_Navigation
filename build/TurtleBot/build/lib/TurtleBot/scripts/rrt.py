import numpy as np
import random
import math
from collections import namedtuple

Node = namedtuple('Node', ['x', 'y', 'parent'])

class RRT:
    def __init__(self, start, goal, map_img, max_iter=1000, step_size=10, goal_sample_rate=0.1):
        self.start = Node(start[0], start[1], None)
        self.goal = Node(goal[0], goal[1], None)
        self.map_img = map_img
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.nodes = [self.start]

    def planning(self):
        for _ in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.nodes, rnd_node)
            new_node = self.steer(nearest_node, rnd_node, self.step_size)
            
            if self.is_collision_free(nearest_node, new_node):
                self.nodes.append(new_node)
                if self.is_near_goal(new_node):
                    final_node = self.steer(new_node, self.goal, self.step_size)
                    if self.is_collision_free(new_node, final_node):
                        return self.generate_final_course(len(self.nodes) - 1)
        
        return None  # No path found

    def get_random_node(self):
        if random.random() > self.goal_sample_rate:
            rnd = Node(random.randint(0, self.map_img.shape[1] - 1),
                       random.randint(0, self.map_img.shape[0] - 1), None)
        else:  # Goal point sampling
            rnd = Node(self.goal.x, self.goal.y, None)
        return rnd

    def get_nearest_node(self, nodes, rnd_node):
        dists = [self.calc_distance(node, rnd_node) for node in nodes]
        nearest_index = dists.index(min(dists))
        return nodes[nearest_index]

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y, from_node)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.x += int(min(extend_length, d) * math.cos(theta))
        new_node.y += int(min(extend_length, d) * math.sin(theta))

        return new_node

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def is_collision_free(self, from_node, to_node):
        if from_node is None or to_node is None:
            return False

        x1, y1 = from_node.x, from_node.y
        x2, y2 = to_node.x, to_node.y

        dx = x2 - x1
        dy = y2 - y1
        steps = int(math.hypot(dx, dy))
        for i in range(steps):
            x = int(x1 + i * dx / steps)
            y = int(y1 + i * dy / steps)
            if self.map_img[y, x] == 0:  # 0 indicates occupied space
                return False

        return True

    def is_near_goal(self, node):
        d = self.calc_distance(node, self.goal)
        return d < self.step_size

    def calc_distance(self, node1, node2):
        return math.hypot(node2.x - node1.x, node2.y - node1.y)

    def generate_final_course(self, goal_index):
        path = [(self.goal.x, self.goal.y)]
        node = self.nodes[goal_index]
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.append((node.x, node.y))
        return path

def find_path_RRT(start, goal, map_img):
    rrt = RRT(start, goal, map_img)
    path = rrt.planning()
    if path is None:
        return [], None
    return path[::-1], rrt.nodes  # Return reversed path and all nodes

