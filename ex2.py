from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import floyd_warshall
from copy import deepcopy
import itertools


ids = ["111111111", "222222222"]

# Objects
DRONES = "drones"
PACKAGES = "packages"
LOCATION = "location"
CLIENTS = "clients"
MAP = "map"
PATH = "path"

# Actions
MOVE = "move"
PICK_UP = "pick up"
DELIVER = "deliver"
WAIT = "wait"
RESET = "reset"
TERMINATE = "terminate"

# Map
PASSABLE = "P"
IMPASSABLE = "I"


class DroneAgent:
    def __init__(self, initial):
        self.map = initial[MAP]
        graph = self.create_map_graph(self.map)
        self.dist_matrix = self.create_dist_matrix(graph)

    def act(self, state):
        my_state = self.convert_state(state)
        print(f"state: {state}")
        print(f"new state: {my_state}\n\n\n")
        return self.greedy_act(my_state)

    def greedy_act(self, my_state):
        pass

    def greedy_act_per_drone(self, drone, my_state):
        drone_location = my_state[DRONES][drone][LOCATION]

        for package in my_state[DRONES][drone][PACKAGES]:
            client_of_package = my_state[PACKAGES][package][CLIENTS]
            client_location = my_state[CLIENTS][client_of_package][LOCATION]
            if drone_location == client_location:
                return (DELIVER, drone, client_of_package, package)

        drone_packages = my_state[DRONES][drone][PACKAGES]
        if len(drone_packages) < 2:
            nearest_package = self.get_nearest_package(drone, my_state)
            package_location = my_state[PACKAGES][nearest_package][LOCATION]
            package_index = self.convert_tuple_to_index(package_location)
            package_distance = self.dist_matrix[self.convert_tuple_to_index(my_state[DRONES][drone][LOCATION])][package_index]
            if package_distance == 0:
                return (PICK_UP, drone, nearest_package)

    def greedy_all_drones(self, my_state):
        all_actions = self.actions(my_state)
        all_results = [self.result(my_state, action) for action in all_actions]



    def get_next_tile_in_path(self, source_index, destination_index):
        pass  # TODO: implement method


    def get_nearest_package(self, drone, my_state):
        packages = my_state[PACKAGES]
        unpicked_packages = filter(lambda p: isinstance(packages[p], tuple), packages.keys())
        packages_with_index = {package: self.convert_tuple_to_index(my_state[PACKAGES][package][LOCATION]) for package in unpicked_packages}
        drone_index = self.convert_tuple_to_index(my_state[DRONES][drone][LOCATION])
        nearest_package = min(packages_with_index, key=lambda p: self.dist_matrix[drone_index][packages_with_index[p]])
        return nearest_package

    def convert_state(self, state):
        my_state = deepcopy(state)
        for drone in state[DRONES]:
            drone_location = state[DRONES][drone]
            my_state[DRONES][drone] = {LOCATION: drone_location,
                                       PACKAGES: []}
        for package in state[PACKAGES]:
            package_location = state[PACKAGES][package]
            my_state[PACKAGES][package] = {LOCATION: package_location}
            if not isinstance(package_location, tuple):  # Package on drone
                my_state[DRONES][package_location][PACKAGES].append(package)

        packages = state[PACKAGES]
        clients = state[CLIENTS]
        for client in clients:
            client_packages = list(clients[client][PACKAGES])
            for package in client_packages:
                my_state[PACKAGES][package][CLIENTS] = client

        return my_state

    def normalize_probabilities(self, client):
        pass  # TODO: implement method

    def create_map_graph(self, problem_map):
        m = len(problem_map)
        n = len(problem_map[0])
        # matrix = [[0 for _ in range(m * n)] for _ in range(m * n)]
        row = []
        col = []
        data = []
        for i in range(m):
            for j in range(n):
                index = self.convert_tuple_to_index((i, j))
                if i < m - 1 and problem_map[i + 1][j] == PASSABLE:
                    lower_index = index + n
                    row.append(index)
                    col.append(lower_index)
                    data.append(1)
                if i > 0 and problem_map[i - 1][j] == PASSABLE:
                    upper_index = index - n
                    row.append(index)
                    col.append(upper_index)
                    data.append(1)
                if j < n - 1 and problem_map[i][j + 1] == PASSABLE:
                    right_index = index + 1
                    row.append(index)
                    col.append(right_index)
                    data.append(1)
                if j > 0 and problem_map[i][j - 1] == PASSABLE:
                    left_index = index - 1
                    row.append(index)
                    col.append(left_index)
                    data.append(1)
        return csr_matrix((data, (row, col)), shape=(m * n, m * n))

    def result(self, my_state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""

        new_state = deepcopy(my_state)

        for atomic_action in action:
            act = atomic_action[0]
            drone = atomic_action[1]
            if act == MOVE:
                new_location = atomic_action[2]
                new_state[DRONES][drone][LOCATION] = new_location
            elif act == PICK_UP:
                package = atomic_action[2]
                new_state[DRONES][drone][PACKAGES].append(package)
                new_state[PACKAGES][package][LOCATION] = drone
            elif act == DELIVER:
                client = atomic_action[2]
                package = atomic_action[3]
                new_state[DRONES][drone][PACKAGES].remove(package)
                new_state[PACKAGES][package][LOCATION] = DELIVER  # TODO: Check if OK (Maybe just delete the package)
                new_state[CLIENTS][client][PACKAGES].remove(package)

        return new_state

    def actions(self, my_state):
        """Returns all the actions that can be executed in the given
        state. The result should be a tuple (or other iterable) of actions
        as defined in the problem description file"""
        drones = my_state[DRONES]
        atomic_actions = []

        for drone in drones:
            drone_atomic_actions = self.get_atomic_actions(drone, my_state)
            atomic_actions.append(drone_atomic_actions)
        actions = filter(self.filter_duplicate_pickups,
                         itertools.product(*atomic_actions))
        return actions

    def filter_duplicate_pickups(self, action):
        package_actions = []
        for atomic_action in action:
            if atomic_action[0] == PICK_UP:
                package_actions.append(atomic_action[2])
        return len(package_actions) == len(set(package_actions))

    def get_atomic_actions(self, drone, my_state):
        """ Get all the possible atomic actions for a specific drone
        :return: Tuple that contains every possible atomic action for the  drone
        """
        drone_object = my_state[DRONES][drone]
        drone_location = drone_object[LOCATION]
        drone_packages = drone_object[PACKAGES]
        drone_data = [drone, drone_location, drone_packages]

        remaining_packages = [package for package in my_state[PACKAGES] if not isinstance(my_state[PACKAGES][package][LOCATION], str)]

        if not drone_packages and not remaining_packages:
            return [(WAIT, drone)]

        possible_atomic_actions = [(WAIT, drone)] #[]#[(WAIT, drone)]
        possible_atomic_actions.extend(
            self.get_deliver_atomic_actions(drone_data, my_state))
        if not possible_atomic_actions:
            possible_atomic_actions.extend(self.get_pickup_atomic_actions(
                drone_data, my_state))
        if not possible_atomic_actions:
            possible_atomic_actions.extend(
                self.get_move_atomic_actions(drone_data))

        # if self.add_wait(my_state, drone) or not possible_atomic_actions:
        #     possible_atomic_actions.append((WAIT, drone))
        # if possible_atomic_actions and all(map((lambda atomic_action: atomic_action[0] == PICK_UP), possible_atomic_actions)):
        #     possible_atomic_actions.append((WAIT, drone))

        return tuple(possible_atomic_actions)

    def get_move_atomic_actions(self, drone_data):
        """ Get all the possible atomic actions the relate to movement for a
        specific drone
        :return: List that contains every possible move atomic action for the
        drone
        """
        drone = drone_data[0]
        location = drone_data[1]
        x = location[0]
        y = location[1]

        move_actions = []
        map_size_x = len(self.map)
        # TODO: make sure we reject a problem  with an empty map
        map_size_y = len(self.map[0])

        if x > 0 and self.map[x - 1][y] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] - 1, location[1])))

        if x < map_size_x - 1 and self.map[x + 1][y] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] + 1, location[1])))

        if y > 0 and self.map[x][y - 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0], location[1] - 1)))

        if y < map_size_y - 1 and self.map[x][y + 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0], location[1] + 1)))

        if x > 0 and y > 0 and self.map[x - 1][y - 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] - 1, location[1] - 1)))

        if x > 0 and y < map_size_y - 1 and self.map[x - 1][y + 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] - 1, location[1] + 1)))

        if x < map_size_x - 1 and y > 0 and self.map[x + 1][y - 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] + 1, location[1] - 1)))

        if x < map_size_x - 1 and y < map_size_y - 1 and self.map[x + 1][y + 1] == PASSABLE:
            move_actions.append((MOVE, drone, (location[0] + 1, location[1] + 1)))

        return move_actions

    def get_deliver_atomic_actions(self, drone_data, my_state):
        """ Get all the possible atomic actions that relate to package delivery
        for a specific drone
        :return: List that contains every possible package delivery atomic
        action for the drone
        """
        drone, location, drone_packages = drone_data
        deliver_actions = []

        clients_on_current_location = self.clients_on_current_location(location, my_state)

        for client in clients_on_current_location:
            package_tuple = my_state[CLIENTS][client][PACKAGES]
            for package in package_tuple:
                if package in drone_packages:
                    deliver_actions.append((DELIVER, drone, client, package))
        return deliver_actions

    def get_pickup_atomic_actions(self, drone_data, state_dict):
        """ Get all the possible atomic actions the relate to package pickup
        for a specific drone
        :return: List that contains every possible package pickup atomic
        action for the drone
        """
        drone, location, drone_packages = drone_data
        pickup_actions = []

        if len(drone_packages) < 2:
            packages_on_current_location = self.packages_on_current_location(location, state_dict)
            for package in packages_on_current_location:
                pickup_actions.append((PICK_UP, drone, package))
        return pickup_actions

    def clients_on_current_location(self, location, my_state):
        """ Get all the clients that currently in a specific location in the map
        :return: List of the clients
        """
        clients_list = []
        for client in my_state[CLIENTS]:
            if my_state[CLIENTS][client][LOCATION] == location:
                clients_list.append(client)

        return clients_list

    def packages_on_current_location(self, location, state_dict):
        package_list = []
        for package in state_dict[PACKAGES]:
            if state_dict[PACKAGES][package][LOCATION] == location:
                package_list.append(package)

        return package_list

    def create_dist_matrix(self, graph):
        dist_matrix, predecessors = floyd_warshall(csgraph=graph, directed=True, return_predecessors=True, unweighted=False)
        return dist_matrix

    def convert_index_to_tuple(self, index, m, n):
        return index // n, index - (index // n) * m

    def convert_tuple_to_index(self, t):
        # m = len(self.map)
        n = len(self.map[0])
        return t[0] * n + t[1]