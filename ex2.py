from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import floyd_warshall
from copy import deepcopy
import itertools
import random
ids = ["111111111", "222222222"]

# Objects
DRONES = "drones"
PACKAGES = "packages"
LOCATION = "location"
CLIENTS = "clients"
MAP = "map"
PATH = "path"
TURNS = "turns to go"

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
        self.points_collected = 0
        self.rounds_played = 0
        self.rounds_left = initial[TURNS]

    def act(self, state):
        my_state = self.convert_state(state)
        return self.greedy_act(my_state)

    def greedy_act(self, my_state):
        # return self.greedy_all_drones(my_state)
        zero_packages_drone_actions = self.send_zero_packages_drones(my_state)
        taken_drones = set(act[1] for act in zero_packages_drone_actions)
        actions = set(zero_packages_drone_actions)
        for drone in my_state[DRONES]:
            if drone not in taken_drones:
                actions.add(self.greedy_act_per_drone(drone, my_state))

        for action in actions:
            if action[0] == DELIVER:
                self.update_problem_variables(actions)
                return actions
        if self.is_reset_recommended(my_state):
            actions = RESET

        self.update_problem_variables(actions)
        return actions

    def update_problem_variables(self, actions):
        self.rounds_left -= 1
        self.rounds_played += 1
        if actions == RESET:
            self.points_collected -= 15
        else:
            for action in actions:
                if action[0] == DELIVER:
                    self.points_collected += 10

    def get_points_per_round_played(self):
        return self.points_collected / (self.rounds_played + 1)

    def get_points_per_round_left(self, my_state):
        if not my_state[PACKAGES]:
            return 0
        points_accumulated = 0
        max_distance = 0
        packages = my_state[PACKAGES]
        unpicked_packages = list(
            filter(lambda p: isinstance(packages[p][LOCATION], tuple), packages.keys()))
        drones = my_state[DRONES]
        drones_without_packages = list(
            filter(lambda d: len(drones[d][PACKAGES]) == 0, drones.keys()))
        drones_index = {drone: self.convert_tuple_to_index(drones[drone][LOCATION]) for drone in
                        drones_without_packages}
        packages_index = {package: self.convert_tuple_to_index(packages[package][LOCATION]) for
                          package in unpicked_packages}
        drone_package_distance = {}
        for drone, package in itertools.product(drones_without_packages, unpicked_packages):
            package_index = packages_index[package]
            drone_index = drones_index[drone]
            drone_package_distance[(drone, package)] = self.dist_matrix[drone_index][package_index]
        left_packages = set(unpicked_packages)
        left_drones = set(drones_without_packages)
        drone_package_sorted = sorted(drone_package_distance,
                                      key=lambda d_p: drone_package_distance[d_p])
        for drone, package in drone_package_sorted:
            if drone in left_drones and package in left_packages:
                package_index = packages_index[package]
                drone_index = drones_index[drone]
                package_distance = self.dist_matrix[drone_index][package_index]
                client = my_state[PACKAGES][package][CLIENTS]
                client_indexx = self.convert_tuple_to_index(my_state[CLIENTS][client][LOCATION])
                distance_from_client = self.dist_matrix[package_index][client_indexx]
                total_distance = package_distance + distance_from_client
                if total_distance < self.rounds_left:
                    max_distance = max(max_distance, total_distance)
                    points_accumulated += 10
        for drone in my_state[DRONES]:
            drone_location = my_state[DRONES][drone][LOCATION]
            drone_index = self.convert_tuple_to_index(drone_location)
            drone_packages = my_state[DRONES][drone][PACKAGES]

            if len(drone_packages) == 1:
                package = drone_packages[0]
                client_of_package = my_state[PACKAGES][package][CLIENTS]
                client_location = my_state[CLIENTS][client_of_package][
                    LOCATION]  # TODO: Add (maybe) probabilities here!
                client_index = self.convert_tuple_to_index(client_location)
                client_distance = self.dist_matrix[drone_index][client_index]
                if client_distance < self.rounds_left:
                    max_distance = max(max_distance, client_distance)
                    points_accumulated += 10

            if len(drone_packages) == 2:
                package1 = drone_packages[0]
                client1 = my_state[PACKAGES][package1][CLIENTS]
                client1_location = my_state[CLIENTS][client1][LOCATION]
                client1_index = self.convert_tuple_to_index(client1_location)
                client1_distance = self.dist_matrix[drone_index][client1_index]

                package2 = drone_packages[1]
                client2 = my_state[PACKAGES][package2][CLIENTS]
                client2_location = my_state[CLIENTS][client2][LOCATION]
                client2_index = self.convert_tuple_to_index(client2_location)
                client2_distance = self.dist_matrix[drone_index][client2_index]

                closest_client = min(client1_distance, client2_distance)
                if closest_client < self.rounds_left:
                    max_distance = max(max_distance, closest_client)
                    points_accumulated += 10

        if max_distance == 0:
            return 500
        return points_accumulated / (max_distance)

    def is_reset_recommended(self, my_state):
        rounds_left = self.rounds_left
        points_per_rounds_played = self.get_points_per_round_played()
        points_per_round_left = self.get_points_per_round_left(my_state)
        if (self.rounds_left * self.get_points_per_round_played() - 5 >
                 self.rounds_left * self.get_points_per_round_left(my_state)):
            return True
        return False

    def client_next_location(self, client, my_state):
        movements = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]
        properties = my_state[CLIENTS][client]
        for _ in range(1000):
            movement = random.choices(movements, weights=properties["probabilities"])[0]
            new_coordinates = (
            properties["location"][0] + movement[0], properties["location"][1] + movement[1])
            if new_coordinates[0] < 0 or new_coordinates[1] < 0 or new_coordinates[0] >= len(
                    my_state["map"]) or \
                    new_coordinates[1] >= len(my_state["map"][0]):
                continue
            break
        else:
            new_coordinates = (properties["location"][0], properties["location"][1])
        return new_coordinates

    def greedy_act_per_drone(self, drone, my_state):
        drone_location = my_state[DRONES][drone][LOCATION]
        drone_index = self.convert_tuple_to_index(drone_location)

        for package in my_state[DRONES][drone][PACKAGES]:
            client_of_package = my_state[PACKAGES][package][CLIENTS]
            client_location = my_state[CLIENTS][client_of_package][LOCATION]
            if drone_location == client_location:
                return (DELIVER, drone, client_of_package, package)

        drone_packages = my_state[DRONES][drone][PACKAGES]

        for package in my_state[PACKAGES]:
            if my_state[PACKAGES][package][LOCATION] == my_state[DRONES][drone][LOCATION] and len(
                    drone_packages) < 2:  # TODO: Check that package has not been taken yet by other drone

                    return (PICK_UP, drone, package)

        if len(drone_packages) == 1:
            package = drone_packages[0]
            client_of_package = my_state[PACKAGES][package][CLIENTS]
            client_location = my_state[CLIENTS][client_of_package][
                LOCATION]  # TODO: Add (maybe) probabilities here!
            client_location = self.client_next_location(client_of_package, my_state)
            client_index = self.convert_tuple_to_index(client_location)
            if client_location == drone_location:
                return (WAIT, drone)
            best_move = self.get_next_tile_in_path(drone, client_index, my_state)
            return (MOVE, drone, best_move)

        if len(drone_packages) == 2:
            package1 = drone_packages[0]
            client1 = my_state[PACKAGES][package1][CLIENTS]
            client1_location = my_state[CLIENTS][client1][LOCATION]
            client1_location = self.client_next_location(client1, my_state)
            client1_index = self.convert_tuple_to_index(client1_location)
            client1_distance = self.dist_matrix[drone_index][client1_index]

            package2 = drone_packages[1]
            client2 = my_state[PACKAGES][package2][CLIENTS]
            client2_location = my_state[CLIENTS][client2][LOCATION]
            client2_location = self.client_next_location(client2, my_state)
            client2_index = self.convert_tuple_to_index(client2_location)
            client2_distance = self.dist_matrix[drone_index][client2_index]

            if client1_distance < client2_distance:
                best_move = self.get_next_tile_in_path(drone, client1_index, my_state)
                if client1_location == drone_location:
                    return (WAIT, drone)
            else:
                if client2_location == drone_location:
                    return (WAIT, drone)
                best_move = self.get_next_tile_in_path(drone, client2_index, my_state)
            return (MOVE, drone, best_move)

    def greedy_all_drones(self, my_state):
        if not my_state[PACKAGES]:
            return RESET
        all_actions = self.actions(my_state)
        all_results = {action: self.result(my_state, action) for action in all_actions}
        min_action = min(all_results, key=lambda act: self.objects_distance_sum_h(all_results[act]))
        # print(my_state[CLIENTS])
        # print(min_action)
        # print()
        # print()
        return min_action

    def objects_distance_sum_h(self, my_state):
        packages = my_state[PACKAGES]
        objects_locations = [packages[package][LOCATION] for package in packages if
                             not isinstance(packages[package][LOCATION], str)]
        clients = my_state[CLIENTS]
        sum = 0
        for client in clients:
            if clients[client][PACKAGES]:
                objects_locations.append(my_state[CLIENTS][client][
                                             LOCATION])  # (self.client_centroids[client])#(client_path[0])
                # sum += len(client_path) - clients_index  # TODO: improved some.

        objects_locations_indexes = [self.convert_tuple_to_index(loc) for loc in objects_locations]

        drones = my_state[DRONES]
        for drone in drones:
            drone_location = drones[drone][LOCATION]
            drone_location_index = self.convert_tuple_to_index(drone_location)
            sum += self.distance_sum_from_location(drone_location_index, objects_locations_indexes)

        return sum

    def distance_sum_from_location(self, location_index, locations_list):
        sum = 0
        for index in locations_list:
            sum += self.dist_matrix[location_index][
                index]  # ** 0.5  # TODO: Improve number of actions, worse time
        return sum

    def get_next_tile_in_path(self, drone, destination_index, my_state):
        pass  # TODO: implement method
        drone_object = my_state[DRONES][drone]
        drone_location = drone_object[LOCATION]
        drone_packages = drone_object[PACKAGES]
        drone_data = [drone, drone_location, drone_packages]
        move_actions = self.get_move_atomic_actions(
            drone_data)  # TODO: maybe include staying in the same place
        best_move = min(move_actions,
                        key=lambda act: self.dist_matrix[self.convert_tuple_to_index(act[2])][
                            destination_index])
        return best_move[2]

    def send_zero_packages_drones(self, my_state):
        packages = my_state[PACKAGES]
        unpicked_packages = list(
            filter(lambda p: isinstance(packages[p][LOCATION], tuple), packages.keys()))
        drones = my_state[DRONES]
        drones_without_packages = list(
            filter(lambda d: len(drones[d][PACKAGES]) == 0, drones.keys()))
        drones_index = {drone: self.convert_tuple_to_index(drones[drone][LOCATION]) for drone in
                        drones_without_packages}
        packages_index = {package: self.convert_tuple_to_index(packages[package][LOCATION]) for
                          package in unpicked_packages}
        drone_package_distance = {}
        for drone, package in itertools.product(drones_without_packages, unpicked_packages):
            package_index = packages_index[package]
            drone_index = drones_index[drone]
            drone_package_distance[(drone, package)] = self.dist_matrix[drone_index][package_index]
        left_packages = set(unpicked_packages)
        left_drones = set(drones_without_packages)
        drone_package_sorted = sorted(drone_package_distance,
                                      key=lambda d_p: drone_package_distance[d_p])
        actions = []
        for drone, package in drone_package_sorted:
            if drone in left_drones and package in left_packages:
                if drone_package_distance[(drone, package)] == 0:
                    actions.append((PICK_UP, drone, package))
                else:
                    package_index = packages_index[package]
                    drone_index = drones_index[drone]
                    best_move = self.get_next_tile_in_path(drone, package_index,
                                                           my_state)  # TODO: implement this method to return tuple(!!)
                    actions.append((MOVE, drone, best_move))
                left_drones.remove(drone)
                left_packages.remove(package)
                if len(left_packages) == 0 or len(left_drones) == 0:
                    break

        for drone in left_drones:  # Left drones need to WAIT
            # print(f"Add WAIT to {drone = }")
            actions.append((WAIT, drone))
        # print(actions)
        return actions

    def get_nearest_package(self, drone, my_state):
        packages = my_state[PACKAGES]
        unpicked_packages = filter(lambda p: isinstance(packages[p][LOCATION], tuple),
                                   packages.keys())
        packages_with_index = {
            package: self.convert_tuple_to_index(my_state[PACKAGES][package][LOCATION]) for package
            in
            unpicked_packages}
        drone_index = self.convert_tuple_to_index(my_state[DRONES][drone][LOCATION])
        nearest_package = min(packages_with_index,
                              key=lambda p: self.dist_matrix[drone_index][packages_with_index[p]])
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
            client_undelivered_packages = []
            for package in client_packages:
                if package in packages:
                    my_state[PACKAGES][package][CLIENTS] = client
                    client_undelivered_packages.append(package)
            my_state[CLIENTS][client][PACKAGES] = client_undelivered_packages

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
                new_state[PACKAGES][package][
                    LOCATION] = DELIVER  # TODO: Check if OK (Maybe just delete the package)
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

        remaining_packages = [package for package in my_state[PACKAGES] if
                              not isinstance(my_state[PACKAGES][package][LOCATION], str)]

        if not drone_packages and not remaining_packages:
            return [(WAIT, drone)]

        possible_atomic_actions = []  # [(WAIT, drone)] #[]#[(WAIT, drone)]
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
        possible_atomic_actions.append((WAIT, drone))
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
        dist_matrix, predecessors = floyd_warshall(csgraph=graph, directed=True,
                                                   return_predecessors=True, unweighted=False)
        return dist_matrix

    def convert_index_to_tuple(self, index, m, n):
        return index // n, index - (index // n) * m

    def convert_tuple_to_index(self, t):
        # m = len(self.map)
        n = len(self.map[0])
        return t[0] * n + t[1]
