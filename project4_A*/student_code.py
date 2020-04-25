import numpy as np

def shortest_path(M,start,goal):
    print("shortest path called")
    close_set = set()
    open_set = {start}
    came_from = {}
    g_score = {}
    infinite = float('inf')
    for each in M.intersections:
        g_score[each] = infinite
    g_score[start] = 0
    f_score = {}
    f_score[start] = heuristic_cost_estimate(M, start, goal)
    while open_set:
        current = lowest_cost_node(f_score, open_set)

        if current == goal:
            return reconstruct_path(came_from, current)
        
        open_set.remove(current)
        close_set.add(current)

        for each in M.roads[current]:
            if each not in close_set and each not in open_set:
                open_set.add(each)
            new_cost = g_score[current] + heuristic_cost_estimate(M, current, each)
            if new_cost < g_score[each]:
                came_from[each] = current
                g_score[each] = new_cost
                f_score[each] = g_score[each] + heuristic_cost_estimate(M, each, goal)
    return False

    
def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path


def lowest_cost_node(f_score, open_set):
    f_open_set = {}
    for node in open_set:
        f_open_set[node] = f_score[node]
    return min(f_open_set, key=f_open_set.get)
    

def heuristic_cost_estimate(M, node1, node2):
    return np.sqrt((M.intersections[node1][0] -M.intersections[node2][0])**2 + (M.intersections[node1][1] - M.intersections[node2][1])**2 )


