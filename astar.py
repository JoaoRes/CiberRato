from heapq import *

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,2),(0,-2),(2,0),(-2,0)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            #data.append(start)
            return [start] + data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < len(array):
                if 0 <= neighbor[1] < len(array[0]):                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
           
    return None

neighbors = [(0,2),(0,-2),(2,0),(-2,0)]
list_visited = [(6, -4), (2, -2), (4, 0), (0, 0), (6, -2), (2, 0), (4, -4), (2, -4), (6, 0)]
list_notTaken = [(6,2)]

for i, j in neighbors:
    neigh = (list_notTaken[0][0]+i, list_notTaken[0][1]+j)
    if neigh in list_visited:
        list_notTaken[0]= neigh 

print(list_notTaken[0])
print(astar(list_visited, (4,-2) ,list_notTaken[0]))