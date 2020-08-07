import pygame
from queue import PriorityQueue
import math

pygame.init()

WIDTH = 700

C_WHITE = (255, 255, 255)
C_START = (255, 165 ,0)
C_END = (67, 16, 98)
C_OPEN = (66, 220, 192)
C_CLOSE = (66, 204, 226)
C_PATH = (255, 253, 105)
C_WALL = (19, 55, 72)
C_GRID_LINES = (192, 223, 249)

win = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")


class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = C_WHITE
        self.neighbours = []
        self.width = width
        self.total_rows = total_rows
    
    def get_pos(self):
        return self.row, self.col
    
    def is_closed(self):
        return self.color == C_CLOSE
    
    def is_open(self):
        return self.color == C_OPEN
    
    def is_obstacle(self):
        return self.color == C_WALL
    
    def is_start(self):
        return self.color == C_START
    
    def is_end(self):
        return self.color == C_END
    
    def make_closed(self):
        self.color = C_CLOSE
    
    def make_open(self):
        self.color = C_OPEN
    
    def make_obstacle(self):
        self.color = C_WALL
    
    def make_start(self):
        self.color = C_START
    
    def make_end(self):
        self.color = C_END

    def make_path(self):
        self.color = C_PATH
    
    def reset(self):
        self.color = C_WHITE

    def draw(self, win):
        self.box = pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))
    
    def update_neighbours(self, grid):
        self.neighbours = []
        if self.row < self.total_rows - 1 and not grid[self.row+1][self.col].is_obstacle():
            self.neighbours.append(grid[self.row+1][self.col])
        if self.row > 0 and not grid[self.row-1][self.col].is_obstacle():
            self.neighbours.append(grid[self.row-1][self.col])
        if self.col < self.total_rows - 1 and not grid[self.row][self.col+1].is_obstacle():
            self.neighbours.append(grid[self.row][self.col+1])
        if self.col > 0 and not grid[self.row][self.col-1].is_obstacle():
            self.neighbours.append(grid[self.row][self.col-1])
        

    def __lt__(self, other):
        return False


def h_score(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2) + abs(y1-y2)


def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


def astar(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = h_score(start.get_pos(), end.get_pos())
    open_set_hash = {start}
    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()
        current = open_set.get()[2]
        open_set_hash.remove(current)
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            start.make_start()
            return True
        for neighbour in current.neighbours:
            temp_g_score = g_score[current] + 1
            if temp_g_score < g_score[neighbour]:
                came_from[neighbour] = current
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score + h_score(neighbour.get_pos(), end.get_pos())
                if neighbour not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbour], count, neighbour))
                    open_set_hash.add((neighbour))
                    neighbour.make_open()
        draw()

        if current != start:
            current.make_closed()
    return False


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid


def draw_grid_lines(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, C_GRID_LINES, (0, i*gap), (width, i*gap))
        for j in range(rows):
            pygame.draw.line(win, C_GRID_LINES, (j*gap, 0), (j*gap, width))


def draw_grid(win, grid, rows, width):
    win.fill(C_WHITE)

    for row in grid:
        for nodes in row:
            nodes.draw(win)
    
    draw_grid_lines(win, rows, width)
    pygame.display.update()


def get_click_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col


def main(win, width):
    ROWS = 50
    grid = make_grid(ROWS, width)
    start = None
    end = None
    started = False
    run = True
    
    while run:
        draw_grid(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end and not started:
                    started = True
                    for row in grid:
                        for nodes in row:
                            nodes.update_neighbours(grid)
                    astar(lambda: draw_grid(win, grid, ROWS, width), grid, start, end)
                if event.key == pygame.K_r:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
                    started = False
            if pygame.mouse.get_pressed()[0] and not started:
                pos = pygame.mouse.get_pos()
                row, col = get_click_pos(pos, ROWS, width)
                node = grid[row][col]
                if not start and node != end:
                    start = node
                    start.make_start()
                elif not end and node != start:
                    end = node
                    end.make_end()
                elif node != end and node != start:
                    node.make_obstacle()
            elif pygame.mouse.get_pressed()[2] and not started:
                pos = pygame.mouse.get_pos()
                row, col = get_click_pos(pos, ROWS, width)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None

    pygame.quit()


main(win, WIDTH)
