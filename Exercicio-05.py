from queue import PriorityQueue
import matplotlib.pyplot as plt
import numpy as np


def manhattan_distance(state, goal):
    distance = 0
    for i in range(1, 9):
        index_state = state.index(i)
        index_goal = goal.index(i)
        distance += abs(index_state // 3 - index_goal // 3) + abs(index_state % 3 - index_goal % 3)
    return distance


def get_possible_moves(index):
    moves = []
    if index % 3 > 0:  # Pode mover para esquerda
        moves.append(-1)
    if index % 3 < 2:  # Pode mover para direita
        moves.append(1)
    if index // 3 > 0:  # Pode mover para cima
        moves.append(-3)
    if index // 3 < 2:  # Pode mover para baixo
        moves.append(3)
    return moves


def get_neighbors(state):
    index_0 = state.index(0)
    possible_moves = get_possible_moves(index_0)
    neighbors = []

    for move in possible_moves:
        new_state = state.copy()
        new_state[index_0], new_state[index_0 + move] = new_state[index_0 + move], new_state[index_0]
        neighbors.append(new_state)

    return neighbors


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from.keys():
        current = came_from[current]
        path.insert(0, current)
    return path


def a_star(initial, goal):
    priority_queue = PriorityQueue()
    initial_priority = manhattan_distance(initial, goal)

    priority_queue.put((initial_priority, 0, initial))
    came_from = {}
    cost_so_far = {tuple(initial): 0}

    while not priority_queue.empty():
        current_priority, current_cost, current = priority_queue.get()

        if current == goal:
            return reconstruct_path(came_from, tuple(current))

        for neighbor in get_neighbors(current):
            new_cost = current_cost + 1
            if tuple(neighbor) not in cost_so_far or new_cost < cost_so_far[tuple(neighbor)]:
                cost_so_far[tuple(neighbor)] = new_cost
                priority = new_cost + manhattan_distance(neighbor, goal)
                priority_queue.put((priority, new_cost, neighbor))
                came_from[tuple(neighbor)] = tuple(current)

    return None


def draw_puzzle(state, step=0):
    puzzle = np.array(state).reshape(3, 3)
    fig, ax = plt.subplots()
    ax.matshow(np.zeros_like(puzzle), cmap='bone', vmin=-1, vmax=2)

    for (i, j), z in np.ndenumerate(puzzle):
        ax.text(j, i, f'{z}', ha='center', va='center', fontsize=24 if z != 0 else 0)

    plt.title(f"Step: {step}")
    plt.axis('off')
    plt.show()


initial_state = [2, 8, 3, 1, 6, 0, 4, 7, 5]
goal_state = [1, 2, 3, 8, 0, 4, 7, 6, 5]

path = a_star(initial_state, goal_state)

if path:
    for step, state in enumerate(path):
        draw_puzzle(state, step)
        # print(path) -> caso queira ver o path percorrido em formato de texto, basta descomentar essa linha.
else:
    print("Could not find a path to the goal.")
