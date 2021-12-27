from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import floyd_warshall
from copy import deepcopy

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
        if len(drone_packages < 2):
            nearest_package = self.get_nearest_package(drone, my_state)
            package_location = my_state[PACKAGES][nearest_package][LOCATION]
            package_index = self.convert_tuple_to_index(package_location)
            package_distance = self.dist_matrix[self.convert_tuple_to_index(my_state[DRONES][drone][LOCATION])][package_index]
            if package_distance == 0:
                return (PICK_UP, drone, nearest_package)


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

    def create_dist_matrix(self, graph):
        dist_matrix, predecessors = floyd_warshall(csgraph=graph, directed=True, return_predecessors=True, unweighted=False)
        return dist_matrix

    def convert_index_to_tuple(self, index, m, n):
        return index // n, index - (index // n) * m

    def convert_tuple_to_index(self, t):
        # m = len(self.map)
        n = len(self.map[0])
        return t[0] * n + t[1]