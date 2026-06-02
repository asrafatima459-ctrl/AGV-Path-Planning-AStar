import heapq
import matplotlib.pyplot as plt
import numpy as np

grid = [
    [0,0,0,0,0],
    [1,1,0,1,0],
    [0,0,0,1,0],
    [0,1,0,0,0],
    [0,0,0,0,0]
]

start = (0,0)
goal = (4,4)

def heuristic(a,b):
    return abs(a[0]-b[0])+abs(a[1]-b[1])

def astar(grid,start,goal):

    rows=len(grid)
    cols=len(grid[0])

    open_set=[]
    heapq.heappush(open_set,(0,start))

    came_from={}
    g={start:0}

    while open_set:

        current=heapq.heappop(open_set)[1]

        if current==goal:

            path=[]

            while current in came_from:
                path.append(current)
                current=came_from[current]

            path.append(start)
            return path[::-1]

        for dx,dy in [(0,1),(1,0),(-1,0),(0,-1)]:

            nx=current[0]+dx
            ny=current[1]+dy

            neighbor=(nx,ny)

            if 0<=nx<rows and 0<=ny<cols:

                if grid[nx][ny]==1:
                    continue

                tentative=g[current]+1

                if neighbor not in g or tentative<g[neighbor]:

                    came_from[neighbor]=current
                    g[neighbor]=tentative

                    f=tentative+heuristic(neighbor,goal)

                    heapq.heappush(
                        open_set,
                        (f,neighbor)
                    )

    return []

path=astar(grid,start,goal)

grid_np=np.array(grid)

plt.imshow(grid_np,cmap='gray_r')

for p in path:
    plt.plot(
        p[1],
        p[0],
        'ro'
    )

plt.title("AGV Path Planning using A*")

plt.savefig("output.png")

plt.show()