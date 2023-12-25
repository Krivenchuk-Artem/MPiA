import tkinter as tk
import random
import math
import heapq


COUNT_OF_SAMPLES = 1000
COUNT_OF_NEIGHBORING_SAMPLES = 10


class PRM:
    def __init__(self, canvas, obstacles, vertices):
        self.canvas = canvas
        self.obstacles = obstacles
        self.vertices = vertices
        self.edges = []

    def random_sample(self):
        x = random.uniform(0, self.canvas.winfo_width())
        y = random.uniform(0, self.canvas.winfo_height())
        return (x, y)

    def collision_free(self, point1, point2):
        for obstacle in self.obstacles:
            if self.line_intersects_rect(point1, point2, obstacle):
                return False
        return True

    def line_intersects_rect(self, p1, p2, rect):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        def intersect(A, B, C, D):
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

        rectangle_corners = [
            (rect[0], rect[1]),
            (rect[2], rect[1]),
            (rect[2], rect[3]),
            (rect[0], rect[3]),
        ]

        line_segments = [
            (rectangle_corners[0], rectangle_corners[1]),
            (rectangle_corners[1], rectangle_corners[2]),
            (rectangle_corners[2], rectangle_corners[3]),
            (rectangle_corners[3], rectangle_corners[0]),
        ]

        for segment in line_segments:
            if intersect(p1, p2, segment[0], segment[1]):
                return True

        return False

    def near(self, q, k):
        distances = [(self.distance(q, v), v) for v in self.vertices]
        distances.sort()
        return [v for _, v in distances[:k]]

    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def prm(self, n, k):
        while len(self.vertices) < n:
            q_rand = self.random_sample()
            if not self.collision_free(self.vertices[0], q_rand):
                continue
            self.vertices.append(q_rand)

        for q in self.vertices[:-1]:
            q_near = self.near(q, k)
            for q_n in q_near:
                if self.collision_free(q, q_n):
                    self.edges.append((q, q_n))
                    self.canvas.create_line(q[0], q[1], q_n[0], q_n[1], fill="gray")

    def find_shortest_path(self, start, goal):
        graph = {v: set() for v in self.vertices}
        for edge in self.edges:
            graph[edge[0]].add((edge[1], self.distance(edge[0], edge[1])))
            graph[edge[1]].add((edge[0], self.distance(edge[1], edge[0])))

        queue = [(0, start, [])]
        visited = set()
        while queue:
            cost, current, path = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            path = path + [current]

            if current == goal:
                return path

            for neighbor, weight in graph[current]:
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + weight, neighbor, path))

        return None


class RobotPathPlanner:
    def __init__(self, root):
        self.root = root
        self.root.title("PRM Path Planning")

        self.canvas = tk.Canvas(root, width=800, height=600, bg="white")
        self.canvas.pack(side=tk.LEFT, expand=tk.YES, fill=tk.BOTH)

        self.mode = None
        self.obstacles = []
        self.start = None
        self.goal = None
        self.intermediate_points = []

        self.create_widgets()
        self.canvas.bind("<Button-1>", self.on_canvas_click)

    def create_widgets(self):
        control_frame = tk.Frame(self.root)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        tk.Button(
            control_frame, text="Задать Старт и Финиш", command=self.set_start_goal_mode
        ).pack(fill=tk.X)
        tk.Button(
            control_frame,
            text="Задать промежуточные точки",
            command=self.set_intermediate_mode,
        ).pack(fill=tk.X)
        tk.Button(
            control_frame, text="Задать препятствия", command=self.set_obstacle_mode
        ).pack(fill=tk.X)
        tk.Button(control_frame, text="Сброс", command=self.reset).pack(fill=tk.X)
        tk.Button(control_frame, text="Старт", command=self.run_prm).pack(fill=tk.X)

    def set_start_goal_mode(self):
        self.mode = "start_goal"

    def set_intermediate_mode(self):
        self.mode = "intermediate"

    def set_obstacle_mode(self):
        self.mode = "obstacle"

    def reset(self):
        self.canvas.delete("all")
        self.obstacles = []
        self.start = None
        self.goal = None
        self.intermediate_points = []

    def on_canvas_click(self, event):
        if self.mode == "start_goal":
            if not self.start:
                self.start = (event.x, event.y)
                self.draw_point(self.start, color="green")
            elif not self.goal:
                self.goal = (event.x, event.y)
                self.draw_point(self.goal, color="red")
            else:
                self.start = (event.x, event.y)
                self.goal = None
                self.canvas.delete("all")
                self.draw_point(self.start, color="green")
        elif self.mode == "intermediate":
            point = (event.x, event.y)
            self.intermediate_points.append(point)
            self.draw_point(point, color="blue")
        elif self.mode == "obstacle":
            obstacle = self.create_obstacle(event.x, event.y)
            self.obstacles.append(obstacle)
            self.draw_obstacle(obstacle)

    def create_obstacle(self, x, y):
        size = 50
        return [x - size / 2, y - size / 2, x + size / 2, y + size / 2]

    def draw_point(self, point, color="black"):
        self.canvas.create_oval(
            point[0] - 5, point[1] - 5, point[0] + 5, point[1] + 5, fill=color
        )

    def draw_obstacle(self, obstacle):
        self.canvas.create_rectangle(
            obstacle[0], obstacle[1], obstacle[2], obstacle[3], fill="gray"
        )

    def run_prm(self):
        if self.start and self.goal:
            self.canvas.delete("all")
            self.draw_point(self.start, color="green")
            self.draw_point(self.goal, color="red")
            for point in self.intermediate_points:
                self.draw_point(point, color="blue")
            for obstacle in self.obstacles:
                self.draw_obstacle(obstacle)

            prm = PRM(
                self.canvas,
                self.obstacles,
                [self.start, self.goal] + self.intermediate_points,
            )
            prm.prm(n=COUNT_OF_SAMPLES, k=COUNT_OF_NEIGHBORING_SAMPLES)

            current_point = self.start
            for point in self.intermediate_points + [self.goal]:
                path = prm.find_shortest_path(current_point, point)
                if path:
                    for i in range(len(path) - 1):
                        self.canvas.create_line(
                            path[i][0],
                            path[i][1],
                            path[i + 1][0],
                            path[i + 1][1],
                            fill="green",
                        )
                    current_point = point
                else:
                    self.canvas.create_text(
                        400,
                        300,
                        text="Составить путь невозможно",
                        font=("Arial", 20),
                        fill="red",
                    )
                    return  # Прерываем выполнение дальнейших действий
        else:
            print("Установите начальную и конечную точку.")


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotPathPlanner(root)
    root.mainloop()
