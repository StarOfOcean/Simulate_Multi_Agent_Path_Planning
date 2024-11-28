import numpy as np
import heapq
from itertools import count
from math import fabs
from copy import deepcopy
from itertools import combinations
from src.simulator.Car import Car
from src.simulator.Position import Position
from config import *
import json
class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))

class Conflict(object):
    VERTEX = 1
    EDGE = 2
    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'

class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])

class Solution:
    def __init__(self, dimension, task_car, current_point, target_point, a_star_max_iter=100):
        self.dimension = dimension
        self.a_star_max_iter = a_star_max_iter

        start_state = State(0, Location(current_point[0], current_point[1]))
        goal_state = State(0, Location(target_point[0], target_point[1]))
        self.agent_dict = {task_car.sn:{'start':start_state, 'goal':goal_state}}

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

        self.moving_obstacles = []

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors


    def get_first_conflict(self, task_car, solution, cars, T):
        max_t = len(solution)
        result = Conflict()
        for t in range(max_t):
            for car in cars:
                if car == task_car:continue
                if t+T > list(car.path_dict.keys())[-1]:
                    car_position = list(car.path_dict.values())[-1]
                else:
                    car_position = car.path_dict[t+T]
                if (solution[t].location.x-car_position[0])**2+(solution[t].location.y-car_position[1])**2 < 10:
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = solution[t].location
                    result.agent_1 = task_car.sn
                    result.agent_2 = car.sn
                    return result


                # for t1, t2 in zip(list(car.path.keys())[-1], list(car.path.keys())[1:]):
                #     if t >= t1 and t < t2:
                #         x1, y1 = car.path[t1].coord[:2]
                #         x2, y2 = car.path[t2].coord[:2]
                #         car_location = ((t-t1)*(x2-x1)/(t2-t1)+x1, (t-t1)*(y2-y1)/(t2-t1)+y1)
                #     if (solution[t].coord[0]-car_location[0])**2+(solution[t].coord[1]-car_location[1])**2 <= 9:
                #         result.time = t
                #         result.type = Conflict.VERTEX
                #         result.location_1 = solution[t].coord
                #         result.agent_1 = task_car
                #         result.agent_2 = car
                #         return result

            # for agent_1, agent_2 in combinations(solution.keys(), 2):
            #     state_1a = self.get_state(agent_1, solution, t)
            #     state_1b = self.get_state(agent_1, solution, t+1)

            #     state_2a = self.get_state(agent_2, solution, t)
            #     state_2b = self.get_state(agent_2, solution, t+1)

            #     if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
            #         result.time = t
            #         result.type = Conflict.EDGE
            #         result.agent_1 = agent_1
            #         result.agent_2 = agent_2
            #         result.location_1 = state_1a.location
            #         result.location_2 = state_1b.location
            #         return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            # constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def get_all_obstacles(self, time):
        all_obs = set()
        for o in self.moving_obstacles:
            if o[2] < 0 and time >= -o[2]:
                all_obs.add((o[0], o[1]))
        return self.obstacles | all_obs

    def state_valid(self, state):
        return state.location.x >= self.dimension[0][0] and state.location.x < self.dimension[1][0] \
            and state.location.y >= self.dimension[0][1] and state.location.y < self.dimension[1][1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints

    def transition_valid(self, state_1, state_2):
        tup_1 = (state_1.location.x, state_1.location.y, state_2.time)
        tup_2 = (state_2.location.x, state_2.location.y, state_1.time)
        if tup_1 in self.moving_obstacles and tup_2 in self.moving_obstacles and \
                self.moving_obstacles[tup_1] == self.moving_obstacles[tup_2]:
            return False
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)


    def is_at_goal(self, state, target_point):
        return state.is_equal_except_time(target_point)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})

    def compute_solution(self, task_agent, current_point, target_node):
        solution = {}
        self.constraints = self.constraint_dict.setdefault(task_agent.sn, Constraints())
        print(self.constraint_dict)
        current_state = State(0, Location(current_point[0], current_point[1]))
        target_state = State(0, Location(target_node[0], target_node[1]))
        local_solution = self.a_star.search(current_state, target_state)
        if not local_solution:
            return False
        return local_solution
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return len(solution)
        return sum([len(path) for path in solution])


class AStar:
    def __init__(self, env):
        def admissible_heuristic(state, target_point):
            return fabs(state.location.x - target_point.location.x) + fabs(state.location.y - target_point.location.y)
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors
        # self.current_time = env.current_time
        self.max_iter = env.a_star_max_iter
        self.iter = 0

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

    def search(self, current_point, target_point):
        """
        low level search
        """
        initial_state = current_point
        step_cost = 1

        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {}
        g_score[initial_state] = 0

        f_score = {}
        h_score = self.admissible_heuristic(initial_state, target_point)
        f_score[initial_state] = h_score

        heap = []
        index = count(0)
        heapq.heappush(heap, (f_score[initial_state], h_score, next(index), initial_state))

        while open_set and (self.max_iter == -1 or self.iter < self.max_iter):
            self.iter = self.iter + 1
            if self.iter == self.max_iter:
                print('Low level A* - Maximum iteration reached')
            #temp_dict = {open_item: f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            #current = min(temp_dict, key=temp_dict.get)
            current = heapq.heappop(heap)[3]

            if self.is_at_goal(current, target_point):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                h_score = self.admissible_heuristic(neighbor, target_point)
                f_score[neighbor] = g_score[neighbor] + h_score
                heapq.heappush(heap, (f_score[neighbor], h_score, next(index), neighbor))
        return False

class Node:
    def __init__(self, coord, ) -> None:
        self.coord = coord
        self.neighbor_nodes = []

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost
        
def CBS(task_car, current_point, target_point, all_cars, T):
    def generate_plan(solution):
        plan = {}
        for idx in range(len(solution)):
            plan[idx+T] = (solution[idx].location.x, solution[idx].location.y, -16)
        # for agent, path in solution.items():
        #     path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
        #     plan[agent] = path_dict_list
        return plan
    open_set = set()
    closed_set = set()
    Path_solution = Solution([(180, 420), (200, 450)], task_car, current_point, target_point)
    start = HighLevelNode()
    start.constraint_dict = {}
    for agent in Path_solution.agent_dict.keys():
        start.constraint_dict[agent] = Constraints()
    start.solution = Path_solution.compute_solution(task_car, current_point, target_point)
    logger.debug(start.solution)
    if not start.solution:
        return {}
    start.cost = Path_solution.compute_solution_cost(start.solution)
    open_set |= {start}
    
    # for car in env.cars:
    #     t = env.current_time
    #     while t < max(car.path.keys()):
    
    # 检查路径与其他车辆路径是否冲突,若冲突，则添加产生冲突的约束，并重新计算路径
    while open_set:
        P = min(open_set)
        open_set -= {P}
        closed_set |= {P}

        Path_solution.constraint_dict = P.constraint_dict
        conflict_dict = Path_solution.get_first_conflict(task_car, P.solution, all_cars, T)
        logger.error(f"conf: {conflict_dict}")
        if not conflict_dict:
            print("Low level CBS - Solution found")

            return generate_plan(P.solution)

        constraint_dict = Path_solution.create_constraints_from_conflict(conflict_dict)
        logger.error(f"constraint: {constraint_dict}")

        for agent in constraint_dict.keys():
            new_node = deepcopy(P)
            new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])
            print(new_node.constraint_dict)
            Path_solution.constraint_dict = new_node.constraint_dict
            new_node.solution = Path_solution.compute_solution(task_car, current_point, target_point)
            if not new_node.solution:
                continue
            new_node.cost = Path_solution.compute_solution_cost(new_node.solution)

            # TODO: ending condition
            if new_node not in closed_set:
                open_set |= {new_node}

if __name__ == "__main__":
    with open(config_path, 'r', encoding="utf-8") as f:
        config = json.load(f)
    cars = [Car(car['magvSn'], Position(car['birthplace']["x"], car['birthplace']["y"], car['birthplace']["z"]), 0, colors[0]) for car in config["taskParam"]["magvParamList"]]
    cars[0].position = Position(190, 431, -16)
    cars[0].path_dict = {0: (190, 431, -16)}
    start_point = (190, 425, -16)
    target_point = (196, 431, -16)
    # target_point = (190, 425, -16)
    print(CBS(cars[1], start_point, target_point, cars, 2))