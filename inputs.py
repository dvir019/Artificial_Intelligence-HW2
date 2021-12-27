small_inputs = [
    {
        "map": [['P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'P'], ],
        "drones": {'drone 1': (3, 3)},
        "packages": {'package 1': (2, 2),
                     'package 2': (1, 1)},
        "clients": {'Alice': {"location": (0, 1),
                              "packages": ('package 1', 'package 2'),
                              "probabilities": (0.6, 0.1, 0.1, 0.1, 0.1)}},
        "turns to go": 100
    },

    {
        "map": [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'], ],
        "drones": {'drone 1': (3, 0),
                   'drone 2': (0, 3)},
        "packages": {'package 1': (3, 4),
                     'package 2': (3, 4)},
        "clients": {'Alice': {"location": (0, 0),
                              "packages": ('package 1',),
                              "probabilities": (0.3, 0.3, 0.15, 0.15, 0.1)},
                    'Bob': {"location": (2, 1),
                            "packages": ('package 2',),
                            "probabilities": (0.15, 0.15, 0.3, 0.3, 0.1)}
                    },
        "turns to go": 150
    },
]
