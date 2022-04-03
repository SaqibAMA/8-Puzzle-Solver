from collections import deque
import copy
import time

class Node:
    def __init__(self, state=None, g=0, h=0, parents=[]):
        self.state = state
        self.g = g
        self.h = h
        self.parents = parents
    
    def get_cost(self):
        return self.g + self.h
    
    def __eq__(self, other):
        return self.state == other.state
    
    def __ne__(self, other):
        return self.state != other.state

    def __str__(self):
        return str(self.state) + " g: " + str(self.g) + " h: " + str(self.h) + " f: " + str(self.get_cost())

    def print_game_board(self):
        print(" -----------")
        print("| " + str(self.state[0][0]) + " | " + str(self.state[0][1]) + " | " + str(self.state[0][2]) + " |")
        print("| " + str(self.state[1][0]) + " | " + str(self.state[1][1]) + " | " + str(self.state[1][2]) + " |")
        print("| " + str(self.state[2][0]) + " | " + str(self.state[2][1]) + " | " + str(self.state[2][2]) + " |")
        print(" -----------")


class PuzzleSolver:
    def __init__(self, initial_state, final_state):
        self.initial_state = initial_state
        self.final_state = final_state

    # Heuristic computation function
    def heuristic(self, state):

        # Find location of any element in a state
        def get_element_location(element, data):
            for i in range(len(data)):
                for j in range(len(data[0])):
                    if data[i][j] == element:
                        return (j, i)
        
        # Iterate over all elements and calculate manhattan displacement
        def get_manhattan_displacements(data):
            total_manhattan_displacements = 0
            for i in range(len(data)):
                for j in range(len(data[0])):
                    if data[i][j] != 0:
                        # print("Checking for element: {}".format(data[i][j]))
                        x, y = get_element_location(data[i][j], self.final_state)
                        # print("Element {} is at location: {} in final state.".format(data[i][j], (x, y)))
                        # print("Manhattan Displacement: {}".format(abs(i - y) + abs(j - x)))
                        total_manhattan_displacements += abs(i - y) + abs(j - x)
            return total_manhattan_displacements

        # Finds out how many tiles are out of place
        def get_misplaced_tiles(data):
            misplaced_tiles = 0
            for i in range(len(data)):
                for j in range(len(data[0]) - 1):
                    if data[i][j] != 0 and data[i][j] != self.final_state[i][j]:
                        misplaced_tiles += 1
            return misplaced_tiles
        
        # Counting the number of inversions
        def get_inversions(data):
            inversions = 0
            for i in range(len(data)):
                for j in range(len(data[0]) - 1):
                    _data = copy.deepcopy(data)
                    if _data[i][j] != 0 and _data[i][j + 1] != 0:

                        previous_manhattan_displacements = get_misplaced_tiles(_data)
                        _data[i][j], _data[i][j + 1] = _data[i][j + 1], _data[i][j]
                        current_manhattan_displacements = get_misplaced_tiles(_data)

                        if abs(previous_manhattan_displacements - current_manhattan_displacements) == 2:
                            inversions += 1

            return inversions
        
        # Calculate heuristic value
        return get_manhattan_displacements(state)
    
    def solve(self):

        def generate_frontier_nodes(node):
            space = (-1, -1)
            for i in range(len(node.state)):
                for j in range(len(node.state[0])):
                    if node.state[i][j] == 0:
                        space = (j, i)
            
            f_cords = [
                (space[0] - 1, space[1]),
                (space[0] + 1, space[1]),
                (space[0], space[1] - 1),
                (space[0], space[1] + 1),
            ]

            f_cords = [fc for fc in f_cords if fc[0] in range(len(node.state[0])) and fc[1] in range(len(node.state))]

            frontier_nodes = []
            new_parents = copy.deepcopy(node.parents)
            new_parents.append(node)

            for f in f_cords:
                new_state = copy.deepcopy(node.state)
                new_state[space[1]][space[0]] = new_state[f[1]][f[0]]
                new_state[f[1]][f[0]] = 0
                frontier_nodes.append(
                    Node(state=new_state, g=node.g + 1, h=self.heuristic(new_state), parents=new_parents)
                )
            
            return frontier_nodes

        open_list = deque()
        close_list = []

        open_list.append(Node(state=self.initial_state, g=0, h=self.heuristic(self.initial_state), parents=[]))

        while len(open_list):
            min_node = min(open_list, key=lambda x: x.get_cost())

            if min_node.state == self.final_state:

                print("Total nodes explored: {}".format(len(close_list)))
                print("Total nodes generated: {}".format(len(open_list) + len(close_list)))

                path = copy.deepcopy(min_node.parents)
                path.append(min_node)
                return path

            frontier_nodes = generate_frontier_nodes(min_node)
            for f in frontier_nodes:
                if f not in open_list and f not in close_list:
                    open_list.append(f)
            
            open_list.remove(min_node)
            close_list.append(min_node)



def main():
    initial_state = [[1, 7, 3], [2, 4, 5], [8, 6, 0]]
    final_state = [[1, 2, 3], [8, 0, 4], [7, 6, 5]]
    
    solver = PuzzleSolver(initial_state, final_state)
    
    start = time.time()
    solution = solver.solve()
    end = time.time()

    print("Time taken: {}".format(end - start))
    print("Total moves needed: {}".format(len(solution)))

    for node in solution:
        node.print_game_board()
        input()

if __name__ == "__main__":
    main()