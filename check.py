import random

from ex2 import DroneAgent, ids
from inputs import small_inputs
import logging
import time
from copy import deepcopy

RESET_PENALTY = 15
DELIVERY_PRICE = 10
TIME_LIMIT = 5

d = {str(i): [] for i in small_inputs}


def initiate_agent(state):
    return DroneAgent(state)


class EndOfGame(Exception):
    pass


class DroneStochasticProblem:
    def __init__(self, an_input):
        self.state = an_input
        self.real_initial_state = deepcopy(self.state)
        self.initial_state = deepcopy(self.state)
        start = time.perf_counter()
        self.agent = initiate_agent(self.state)
        end = time.perf_counter()
        if end - start > TIME_LIMIT:
            logging.critical("timed out on constructor")
            raise TimeoutError
        self.score = 0

    def run_round(self):
        # from drones_sim_ploter_empty import Plotter
        # plotter = Plotter(self.initial_state)
        # plotter.do_your_thing(self.initial_state)
        while self.state["turns to go"]:
            start = time.perf_counter()
            action = self.agent.act(self.state)
            end = time.perf_counter()
            if end - start > TIME_LIMIT:
                logging.critical(f"timed out on an action")
                raise TimeoutError
            if not self.is_action_legal(action):
                logging.critical(f"You returned an illegal action!")
                raise RuntimeError
            self.result(action)
            # plotter.do_your_thing(self.state)
        self.terminate_execution()

    def is_action_legal(self, action):
        if action == "reset":
            return True
        if action == "terminate":
            return True
        if len(action) != len(self.state["drones"]):
            logging.error(f"You had given {len(action)} atomic commands, while there are {len(self.state['drones'])}"
                          f" drones in the problem!")
            return False
        drones_already_moved = set()
        for atomic_action in action:
            if not self.is_atomic_action_legal(atomic_action, drones_already_moved):
                logging.error(f"Atomic action {atomic_action} is illegal!")
                return False
        return True

    def is_atomic_action_legal(self, atomic_action, drones_already_moved):
        try:
            action_name = atomic_action[0]
            drone_name = atomic_action[1]
        except TypeError:
            logging.error(f"Your atomic action {atomic_action} has wrong syntax!")
            return False

        if drone_name not in self.state["drones"]:
            logging.error(f"Drone {drone_name} does not exist!")
            return False

        if drone_name in drones_already_moved:
            logging.error(f"Drone {drone_name} was already given command on this turn!")
            return False
        drones_already_moved.add(drone_name)

        if action_name == "wait":
            if len(atomic_action) != 2:
                logging.error(f"Your atomic action {atomic_action} has a wrong syntax!")
                return False
            return True

        if action_name == "pick up":
            if len(atomic_action) != 3:
                logging.error(f"Your atomic action {atomic_action} has a wrong syntax!")
                return False
            package = atomic_action[2]
            if self.state["drones"][drone_name] != self.state["packages"][package]:
                logging.error(f"{drone_name} is not in the same tile as {package}!")
                return False
            return True

        if action_name == "move":
            if len(atomic_action) != 3:
                logging.error(f"Your atomic action {atomic_action} has a wrong syntax!")
                return False
            try:
                origin = self.state["drones"][drone_name]
                destination = atomic_action[2]
                if destination[0] < 0 or destination[1] < 0 or destination[0] >= len(self.state["map"]) or destination[1] >= len(self.state["map"][0]):
                    logging.error(f"You are trying to move to {destination}, which is outside of the grid bounds!")
                    return False
                if self.state["map"][destination[0]][destination[1]] == "I":
                    logging.error(f"You are trying to move to {destination}, which is impassable for drones!")
                    return False
                if abs(origin[0] - destination[0]) > 1 or abs(origin[1] - destination[1]) > 1:
                    logging.error(f"You are trying to move from {origin} to {destination}, which is too far!")
                    return False
            except TypeError:
                logging.error(f"Your atomic action {atomic_action} has a wrong syntax!")
                return False
            return True

        if action_name == "deliver":
            if len(atomic_action) != 4:
                logging.error(f"Your atomic action {atomic_action} has a wrong syntax!")
                return False
            client = atomic_action[2]
            package = atomic_action[3]
            if (self.state["drones"][drone_name] != self.state["clients"][client]["location"]) or \
                    (self.state["packages"][package] != drone_name):
                logging.error(f"{drone_name}, {client}, and {package} are not in the same place!")
                return False
            return True

        return False

    def result(self, action):
        self.apply(action)
        if action != "reset":
            self.environment_step()

    def apply(self, action):
        if action == "reset":
            self.reset_environment()
            return
        if action == "terminate":
            self.terminate_execution()
        for atomic_action in action:
            self.apply_atomic_action(atomic_action)

    def apply_atomic_action(self, atomic_action):
        action_name = atomic_action[0]
        drone_name = atomic_action[1]

        if action_name == "wait":
            return

        if action_name == "pick up":
            package = atomic_action[2]
            self.state["packages"][package] = drone_name

        if action_name == "move":
            destination = atomic_action[2]
            self.state["drones"][drone_name] = destination

        if action_name == "deliver":
            package = atomic_action[3]
            self.state["packages"].pop(package)
            self.score += DELIVERY_PRICE

    def environment_step(self):
        movements = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]
        for client, properties in self.state["clients"].items():
            for _ in range(1000):
                movement = random.choices(movements, weights=properties["probabilities"])[0]
                new_coordinates = (properties["location"][0] + movement[0], properties["location"][1] + movement[1])
                if new_coordinates[0] < 0 or new_coordinates[1] < 0 or new_coordinates[0] >= len(self.state["map"]) or new_coordinates[1] >= len(self.state["map"][0]):
                    continue
                break
            else:
                new_coordinates = (properties["location"][0], properties["location"][1])
            assert new_coordinates
            properties["location"] = new_coordinates
        self.state["turns to go"] -= 1

    def reset_environment(self):
        self.state["packages"] = deepcopy(self.initial_state["packages"])
        self.state["clients"] = deepcopy(self.initial_state["clients"])
        self.state["drones"] = deepcopy(self.initial_state["drones"])
        self.score -= RESET_PENALTY
        self.state["turns to go"] -= 1

    def terminate_execution(self):
        # print(f"End of game, your score is {self.score}!")
        # print(f"-----------------------------------")
        d[str(self.real_initial_state)].append(self.score)
        raise EndOfGame


def main():
    import pandas as pd
    print(f"IDS: {ids}")
    start = time.time()
    for i in range(1000):
        small_inputs2 = deepcopy(small_inputs)
        for an_input in small_inputs2:
            try:
                my_problem = DroneStochasticProblem(an_input)
                my_problem.run_round()
            except EndOfGame:
                continue
    for v in d: print(d[v])
    print()
    for v in d:
        df_describe1 = pd.DataFrame(d[v])
        print(df_describe1.describe())
        print()
    print(f"took {time.time() - start} s")
    print()



if __name__ == '__main__':
    main()
    # a = [25, 35, 35, 30, 40, 35, 35, 35, 35, 30, 45, 45, 35, 40, 35, 45, 35, 40, 50, 45, 40, 40, 25, 35, 30, 35, 30, 40, 25, 35, 25, 30, 25, 25, 40, 40, 45, 55, 40, 50, 35, 25, 25, 30, 30, 50, 35, 45, 45, 50, 35, 45, 30, 45, 45, 45, 35, 25, 35, 40, 35, 40, 25, 45, 30, 35, 15, 35, 30, 35, 40, 35, 25, 30, 30, 50, 30, 40, 35, 35, 40, 40, 40, 35, 40, 45, 40, 50, 30, 45, 30, 40, 30, 40, 20, 35, 45, 40, 50, 35]
    # b = [5, 5, 5, 10, 15, 10, 10, 15, 15, 10, 20, 10, 10, 25, 15, 5, 10, 15, 20, 10, 10, 20, 20, 10, 15, 15, 15, 15, 15, 10, 10, 15, 15, 15, 10, 10, 10, 20, 15, 25, 10, 10, 10, 10, 5, 10, 15, 15, 5, 10, 10, 15, 25, 10, 5, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 5, 15, 15, 10, 15, 5, 20, 25, 10, 15, 10, 20, 15, 15, 10, 10, 15, 25, 15, 5, 10, 5, 5, 10, 25, 5, 5, 20, 15, 20, 15, 5, 10]
    # import pandas as pd
    # df_describe1 = pd.DataFrame(a)
    # print(df_describe1.describe())
    # df_describe2 = pd.DataFrame(b)
    # print(df_describe2.describe())
    # print()