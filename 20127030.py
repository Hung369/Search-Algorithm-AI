import turtle
import time
from queue import PriorityQueue
from copy import copy, deepcopy


def readGraph(filename, type, obj):
    myfile = open(filename, 'r')
    element = myfile.read()
    myfile.close()
    lists = element.split('\n')
    length = int(lists[0].split()[0])  # length
    width = int(lists[0].split()[1])  # width

    # coordination of start point
    sx = int(lists[1].split()[0])
    sy = int(lists[1].split()[1])
    start = (sx, sy)

    # coordination of end point
    dx = int(lists[1].split()[2])
    dy = int(lists[1].split()[3])
    destination = (dx, dy)

    # number of obstacles
    num = int(lists[2])

    arguments = []
    for i in range(3, 6):
        name = lists[i].split()
        arguments.append(name)

    for i in range(len(arguments)):
        for j in range(len(arguments[i])):
            if arguments[i][j] != ' ':
                arguments[i][j] = int(arguments[i][j])

    board = CreateMap(length, width, start, destination, num, arguments)
    DrawMap(length, width, obj, start, destination, num, arguments)

    if type == 1:
        BFS(start, destination, board, obj, width, length)
    elif type == 2:
        UCS(start, destination, board, obj, width, length)
    elif type == 3:
        IDS(start, destination, board, obj, width, length)
    elif type == 4:
        GreedyBFS(start, destination, board, obj, width, length)
    elif type == 5:
        A_heuristics(start, destination, board, obj, width, length)
    time.sleep(3)



def CreateMap(length, width, start, destination, n, polygon):
    squarebox = [['*' for i in range(width - 1)] for j in range(length - 1)]  # width columns, length row
    squarebox[start[0] - 1][start[1] - 1] = 'S'
    squarebox[destination[0] - 1][destination[1] - 1] = 'D'

    for num in range(n):
        i = 0
        while i < len(polygon[num]):
            x = polygon[num][i]
            y = polygon[num][i + 1]
            squarebox[x - 1][y - 1] = 'O'
            i += 2

    # ve da giac trong ma tran
    x1 = polygon[0][0] - 1
    y1 = polygon[0][1] - 1

    x2 = polygon[0][2] - 1
    y2 = polygon[0][3] - 1

    x3 = polygon[0][4] - 1
    y3 = polygon[0][5] - 1

    x4 = polygon[0][6] - 1
    y4 = polygon[0][7] - 1

    for i in range(y1 + 1, y2):
        squarebox[x1][i] = 'O'

    squarebox[x2 + 1][y2] = 'O'
    squarebox[x3 - 1][y3] = 'O'
    squarebox[x4 - 1][y4] = 'O'

    for i in range(x1 + 1, x4):
        squarebox[i][y1] = 'O'

    for i in range(y4 + 1, y3):
        squarebox[x4][i] = 'O'

    # ve hinh vuong trong ma tran
    x1 = polygon[2][0] - 1
    y1 = polygon[2][1] - 1

    x2 = polygon[2][2] - 1
    y2 = polygon[2][3] - 1

    x3 = polygon[2][4] - 1
    x4 = polygon[2][6] - 1

    for i in range(y1 + 1, y2):
        squarebox[x1][i] = 'O'
        squarebox[x4][i] = 'O'

    for i in range(x2 + 1, x3):
        squarebox[i][y2] = 'O'
        squarebox[i][y1] = 'O'

    # ve hinh tam giac trong ma tran
    x1 = polygon[1][0] - 1
    y1 = polygon[1][1] - 1

    x2 = polygon[1][2] - 1
    y2 = polygon[1][3] - 1

    x3 = polygon[1][4] - 1
    y3 = polygon[1][5] - 1

    for i in range(y1 + 1, y2):
        squarebox[x1][i] = 'O'

    for i in range(x1 + 1, x3):
        squarebox[i][y1] = 'O'

    c, r = x2, y2
    while c != x3 and r != y3:
        c += 1
        r -= 1
        squarebox[c][r] = 'O'

    return squarebox


def DrawMap(l, w, obj, start, destination, num, polygon):
    # mock
    x = 360
    y = 250
    clen = 36
    cwid = 27
    obj.speed(0)

    for i in range(w + 1):
        obj.penup()
        obj.goto(-x, y)
        y -= cwid
        obj.pendown()

        for j in range(l + 1):
            if i == 0 or i == w or j == 0 or j == l:
                obj.fillcolor('dark gray')
                obj.begin_fill()
            for m in range(2):
                obj.forward(clen)
                obj.right(90)
                obj.forward(cwid)
                obj.right(90)
            obj.forward(clen)
            obj.end_fill()

    # start point marking
    obj.penup()
    y = 250

    obj.goto(-x + clen * start[0], y - cwid * (w + 1) + (start[1] + 1) * cwid)
    obj.pendown()
    obj.fillcolor('yellow')
    obj.begin_fill()
    for k in range(2):
        obj.forward(clen)
        obj.right(90)
        obj.forward(cwid)
        obj.right(90)
    obj.forward(clen)
    obj.end_fill()

    # destination point marking
    obj.penup()
    obj.goto(-x + clen * destination[0], y - cwid * (w + 1) + (destination[1] + 1) * cwid)
    obj.pendown()
    obj.fillcolor('green')
    obj.begin_fill()
    for k in range(2):
        obj.forward(clen)
        obj.right(90)
        obj.forward(cwid)
        obj.right(90)
    obj.forward(clen)
    obj.end_fill()
    obj.penup()

    for n in range(num):
        i = 0
        while i < len(polygon[n]):
            xp = polygon[n][i]
            yp = polygon[n][i + 1]
            obj.goto(-x + clen * xp, y - cwid * (w + 1) + (yp + 1) * cwid)
            obj.pendown()
            obj.fillcolor('blue')
            obj.begin_fill()

            for j in range(2):
                obj.forward(36)
                obj.right(90)
                obj.forward(27)
                obj.right(90)
            obj.forward(36)

            obj.end_fill()
            obj.penup()
            i += 2

    DrawPolyGonShape(polygon, obj, x, y, clen, cwid, w)
    DrawSquare(polygon, obj, x, y, clen, cwid, w)
    DrawTriangle(polygon, obj, x, y, clen, cwid, w)


def DrawPolyGonShape(polygon, obj, x, y, clen, cwid, w):
    x1 = polygon[0][0]
    y1 = polygon[0][1]

    x2 = polygon[0][2]
    y2 = polygon[0][3]

    x3 = polygon[0][4]
    y3 = polygon[0][5]

    x4 = polygon[0][6]
    y4 = polygon[0][7]

    obj.goto(-x + clen * (x2 + 1), y - cwid * (w + 1) + (y2 + 1) * cwid)
    Draw(obj)

    obj.goto(-x + clen * (x3 - 1), y - cwid * (w + 1) + (y3 + 1) * cwid)
    Draw(obj)

    obj.goto(-x + clen * (x4 - 1), y - cwid * (w + 1) + (y4 + 1) * cwid)
    Draw(obj)

    for i in range(y1 + 1, y2):
        obj.goto(-x + clen * x1, y - cwid * (w + 1) + (i + 1) * cwid)
        Draw(obj)

    for i in range(x1 + 1, x4):
        obj.goto(-x + clen * i, y - cwid * (w + 1) + (y1 + 1) * cwid)
        Draw(obj)

    for i in range(y4 + 1, y3):
        obj.goto(-x + clen * x4, y - cwid * (w + 1) + (i + 1) * cwid)
        Draw(obj)


def DrawSquare(polygon, obj, x, y, clen, cwid, w):
    x1 = polygon[2][0]
    y1 = polygon[2][1]

    x2 = polygon[2][2]
    y2 = polygon[2][3]

    x3 = polygon[2][4]
    x4 = polygon[2][6]

    for i in range(y1 + 1, y2):
        obj.goto(-x + clen * x1, y - cwid * (w + 1) + (i + 1) * cwid)
        Draw(obj)
        obj.goto(-x + clen * x4, y - cwid * (w + 1) + (i + 1) * cwid)
        Draw(obj)

    for i in range(x2 + 1, x3):
        obj.goto(-x + clen * i, y - cwid * (w + 1) + (y2 + 1) * cwid)
        Draw(obj)
        obj.goto(-x + clen * i, y - cwid * (w + 1) + (y1 + 1) * cwid)
        Draw(obj)


def DrawTriangle(polygon, obj, x, y, clen, cwid, w):
    x1 = polygon[1][0]
    y1 = polygon[1][1]

    x2 = polygon[1][2]
    y2 = polygon[1][3]

    x3 = polygon[1][4]
    y3 = polygon[1][5]

    for i in range(y1 + 1, y2):
        obj.goto(-x + clen * x1, y - cwid * (w + 1) + (i + 1) * cwid)
        Draw(obj)

    for i in range(x1 + 1, x3):
        obj.goto(-x + clen * i, y - cwid * (w + 1) + (y1 + 1) * cwid)
        Draw(obj)

    c, r = x2, y2
    while c != x3 and r != y3:
        c += 1
        r -= 1
        obj.goto(-x + clen * c, y - cwid * (w + 1) + (r + 1) * cwid)
        Draw(obj)


def Draw(obj):
    obj.pendown()
    obj.fillcolor('blue')
    obj.begin_fill()
    for k in range(2):
        obj.forward(36)
        obj.right(90)
        obj.forward(27)
        obj.right(90)
    obj.forward(36)
    obj.end_fill()
    obj.penup()


def Print(board):
    for i in board:
        print(i)


# algorithms
# BFS
def BFS(start, destination, board, obj, width, length):
    l = []
    explored = []
    moving = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    l.append(start)
    cost = 0
    cost_explored = 0
    parent = {}
    parent[start] = None

    while True:
        if len(l) == 0:
            print('No solution')
            return
        else:
            node = l.pop(0)
            explored.append(node)
            cost_explored += 1

            if node == destination:

                board[start[0] - 1][start[1] - 1] = 'S'
                board[destination[0] - 1][destination[1] - 1] = 'D'

                pathSolution = []
                PathWay(parent, destination, pathSolution)

                for i in pathSolution:
                    cost += 1

                print('BFS cost: ', cost - 1)
                print('Explored cost: ', cost_explored - 1)
                print('Expanded nodes result:')
                Print(board)

                x = 360
                y = 250
                clen = 36
                cwid = 27

                pathSolution.remove(start)
                pathSolution.remove(destination)

                for i in range(len(pathSolution)):
                    obj.goto(-x + clen * pathSolution[i][0], y - cwid * (width + 1) + (pathSolution[i][1] + 1) * cwid)
                    obj.pendown()
                    obj.fillcolor('red')
                    obj.begin_fill()

                    for k in range(2):
                        obj.forward(clen)
                        obj.right(90)
                        obj.forward(cwid)
                        obj.right(90)
                    obj.forward(clen)

                    obj.end_fill()
                    obj.penup()

                break
            else:
                for direct in moving:
                    hoanh, tung = node[0] + direct[0] - 1, node[1] + direct[1] - 1

                    if isValid(hoanh, tung, length, width, board):
                        board[hoanh][tung] = '1'
                        adjacent = (hoanh + 1, tung + 1)
                        if adjacent not in explored:
                            parent[adjacent] = node
                            l.append(adjacent)


# find path from start to end
def PathWay(parent, destination, path):
    path.append(destination)
    while parent[destination] is not None:
        path.append(parent[destination])
        destination = parent[destination]
    path.reverse()


# check if the cell in board is a part of obstacle, visited or out of range?
def isValid(x, y, length, width, board):
    if x < 0 or y < 0 or x >= length - 1 or y >= width - 1 or board[x][y] == 'O' or board[x][y] == '1':
        return False
    else:
        return True


# Uniform Cost Search
def UCS(start, destination, board, obj, width, length):
    l = PriorityQueue()
    explored = PriorityQueue()
    moving = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    cost = 0
    cost_explored = 0
    parent = {}
    cost_parent = {}
    parent[start] = None
    cost_parent[start] = 0
    l.put((0, start))

    while True:
        if l.empty():
            print('No solution')
            break
        else:
            node = l.get()
            explored.put(node)
            cost_explored += 1

            if node[1] == destination:

                board[start[0] - 1][start[1] - 1] = 'S'
                board[destination[0] - 1][destination[1] - 1] = 'D'

                pathSolution = []
                PathWay(parent, destination, pathSolution)

                for i in pathSolution:
                    cost += 1

                print('UCS cost: ', cost - 1)
                print('Explored cost: ', cost_explored - 1)
                print('Expanded nodes result:')
                Print(board)

                x = 360
                y = 250
                clen = 36
                cwid = 27

                pathSolution.remove(start)
                pathSolution.remove(destination)

                for i in range(len(pathSolution)):
                    obj.goto(-x + clen * pathSolution[i][0],
                             y - cwid * (width + 1) + (pathSolution[i][1] + 1) * cwid)
                    obj.pendown()
                    obj.fillcolor('red')
                    obj.begin_fill()

                    for k in range(2):
                        obj.forward(clen)
                        obj.right(90)
                        obj.forward(cwid)
                        obj.right(90)
                    obj.forward(clen)

                    obj.end_fill()
                    obj.penup()

                break
            else:
                for direct in moving:
                    hoanh, tung = node[1][0] + direct[0] - 1, node[1][1] + direct[1] - 1
                    if isValid(hoanh, tung, length, width, board):
                        adjacent = (hoanh + 1, tung + 1)

                        if adjacent not in explored.queue:
                            tmp_cost = node[0] + 1
                            if adjacent not in cost_parent or tmp_cost < cost_parent[adjacent]:
                                board[hoanh][tung] = '1'
                                l.put((tmp_cost, adjacent))
                                cost_parent[adjacent] = tmp_cost
                                parent[adjacent] = node[1]


# Greedy Best First Search
def GreedyBFS(start, destination, board, obj, width, length):
    l = PriorityQueue()
    explored = PriorityQueue()
    moving = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    cost = 0
    cost_explored = 0
    parent = {}
    cost_parent = {}
    parent[start] = None
    cost_parent[start] = 0
    l.put((0, start))

    while True:
        if l.empty():
            print('No solution')
            break
        else:
            node = l.get()
            explored.put(node)
            cost_explored += 1

            if node[1] == destination:

                board[start[0] - 1][start[1] - 1] = 'S'
                board[destination[0] - 1][destination[1] - 1] = 'D'

                pathSolution = []
                PathWay(parent, destination, pathSolution)

                for i in pathSolution:
                    cost += 1

                print('GBFS cost: ', cost - 1)
                print('Explored cost: ', cost_explored - 1)
                print('Expanded nodes result:')
                Print(board)

                x = 360
                y = 250
                clen = 36
                cwid = 27

                pathSolution.remove(start)
                pathSolution.remove(destination)

                for i in range(len(pathSolution)):
                    obj.goto(-x + clen * pathSolution[i][0],
                             y - cwid * (width + 1) + (pathSolution[i][1] + 1) * cwid)
                    obj.pendown()
                    obj.fillcolor('red')
                    obj.begin_fill()

                    for k in range(2):
                        obj.forward(clen)
                        obj.right(90)
                        obj.forward(cwid)
                        obj.right(90)
                    obj.forward(clen)

                    obj.end_fill()
                    obj.penup()

                break
            else:
                for direct in moving:
                    hoanh, tung = node[1][0] + direct[0] - 1, node[1][1] + direct[1] - 1
                    if isValid(hoanh, tung, length, width, board):
                        adjacent = (hoanh + 1, tung + 1)

                        if adjacent not in explored.queue:
                            tmp_cost = Distance_Mat(adjacent, destination)

                            if adjacent not in cost_parent or tmp_cost < cost_parent[adjacent]:
                                board[hoanh][tung] = '1'
                                l.put((tmp_cost, adjacent))
                                cost_parent[adjacent] = tmp_cost
                                parent[adjacent] = node[1]


def Distance_Mat(adjacent, point):
    xdistance = abs(adjacent[0] - point[0])
    ydistance = abs(adjacent[1] - point[1])
    total = xdistance + ydistance
    return total


# A star
def A_heuristics(start, destination, board, obj, width, length):
    l = PriorityQueue()
    explored = PriorityQueue()
    moving = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    cost = 0
    cost_explored = 0
    parent = {}
    cost_parent = {}
    parent[start] = None
    cost_parent[start] = 0
    l.put((0, start))

    while True:
        if l.empty():
            print('No solution')
            break
        else:
            node = l.get()
            explored.put(node)
            cost_explored += 1

            if node[1] == destination:

                board[start[0] - 1][start[1] - 1] = 'S'
                board[destination[0] - 1][destination[1] - 1] = 'D'

                pathSolution = []
                PathWay(parent, destination, pathSolution)

                for i in pathSolution:
                    cost += 1

                print('A-star cost: ', cost - 1)
                print('Explored cost: ', cost_explored - 1)
                print('Expanded result.')
                Print(board)

                x = 360
                y = 250
                clen = 36
                cwid = 27

                pathSolution.remove(start)
                pathSolution.remove(destination)

                for i in range(len(pathSolution)):
                    obj.goto(-x + clen * pathSolution[i][0],
                             y - cwid * (width + 1) + (pathSolution[i][1] + 1) * cwid)
                    obj.pendown()
                    obj.fillcolor('red')
                    obj.begin_fill()

                    for k in range(2):
                        obj.forward(clen)
                        obj.right(90)
                        obj.forward(cwid)
                        obj.right(90)
                    obj.forward(clen)

                    obj.end_fill()
                    obj.penup()

                break
            else:
                for direct in moving:
                    hoanh, tung = node[1][0] + direct[0] - 1, node[1][1] + direct[1] - 1
                    if isValid(hoanh, tung, length, width, board):
                        adjacent = (hoanh + 1, tung + 1)

                        if adjacent not in explored.queue:
                            g = Distance_Mat(node[1], adjacent)
                            h = Distance_Mat(adjacent, destination)
                            f = g + h
                            tmp_cost = cost_parent[node[1]] + f

                            if adjacent not in cost_parent or tmp_cost < cost_parent[adjacent]:
                                board[hoanh][tung] = '1'
                                l.put((tmp_cost, adjacent))
                                cost_parent[adjacent] = tmp_cost
                                parent[adjacent] = node[1]


# Iterative Deepening Search
def IDS(start, destination, board, obj, width, length):
    depth = 0  # muc do sau
    duplicate = deepcopy(board)

    while True:

        if depth >= length - 1 and depth >= width - 1:
            print('No solution')
            break
        else:
            if not LimitDFS(start, destination, board, obj, width, length, depth):
                depth += 1
                board = deepcopy(duplicate)
            else:
                break

# DFS with depth level
def LimitDFS(start, destination, board, obj, width, length, depth):
    l = []
    explored = []
    moving = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    l.append(start)
    cost = 0
    cost_explored = 0
    parent = {}
    parent[start] = None

    while True:
        if len(l) == 0:
            return False
        elif depth == 0:
            return False
        else:
            node = l.pop(0)
            explored.append(node)
            cost_explored += 1

            if node == destination:

                board[start[0] - 1][start[1] - 1] = 'S'
                board[destination[0] - 1][destination[1] - 1] = 'D'

                pathSolution = []
                PathWay(parent, destination, pathSolution)

                for i in pathSolution:
                    cost += 1

                print('IDS cost: ', cost - 1)
                print('Explored cost: ', cost_explored - 1)
                print('Depth: ', depth)
                print('Expanded nodes result:')
                Print(board)

                x = 360
                y = 250
                clen = 36
                cwid = 27

                pathSolution.remove(start)
                pathSolution.remove(destination)

                for i in range(len(pathSolution)):
                    obj.goto(-x + clen * pathSolution[i][0], y - cwid * (width + 1) + (pathSolution[i][1] + 1) * cwid)
                    obj.pendown()
                    obj.fillcolor('red')
                    obj.begin_fill()

                    for k in range(2):
                        obj.forward(clen)
                        obj.right(90)
                        obj.forward(cwid)
                        obj.right(90)
                    obj.forward(clen)

                    obj.end_fill()
                    obj.penup()

                return True
            else:
                for direct in moving:
                    hoanh, tung = node[0] + direct[0] - 1, node[1] + direct[1] - 1

                    if isValidDFS(hoanh, tung, length, width, board, depth):

                        board[hoanh][tung] = '1'
                        adjacent = (hoanh + 1, tung + 1)

                        if adjacent not in explored:
                            parent[adjacent] = node
                            l.insert(0, adjacent)

# check conditions for DFS (a part of obstacle, visited or out of range?)
def isValidDFS(x, y, length, width, board,deep):
    if (x < 0 or y < 0 or x >= length - 1 or y >= width - 1 or x > deep or y > deep
            or board[x][y] == 'O' or board[x][y] == '1'):
        return False
    else:
        return True


if __name__ == '__main__':
    filename = 'input.txt'
    print('-------- LAB 01: SEARCH --------')
    print('1. Breadth-first search')
    print('2. Uniform-cost search')
    print('3. Iterative deepening search')
    print('4. Greedy-best first search')
    print('5. Graph-search A*')
    type = int(input('Please choose an algorithm (0 to Exit): '))
    while type not in range(6):
        type = int(input('Please choose algorithm (0 to Exit): '))
    obj = turtle.Turtle()

    if type != 0:
        readGraph(filename, type, obj)
    else:
        print('Exit')