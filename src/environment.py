import os
import simplejson
import numpy as np
import matplotlib.pyplot as plt


class Point_Object:
    def __init__(self, position, is_sink, collision_radius, field_radius):
        self.position = position
        self.collision_radius = collision_radius
        self.field_radius = field_radius

        if is_sink: # attract
            self.multiplier = 1
        else: # repel
            self.multiplier = -1


    def distance(self, position):
        '''compute the distance from center to position'''

        v = self.position - position
        return np.sqrt(np.dot(v, v))


    def is_in_collision(self, position):
        '''check if position is in collision with this instance'''

        if self.collision_radius == 0:
            return False

        if self.distance(position) < self.collision_radius:
            return True
        return False


    def is_intersecting(self, position_1, position_2):
        '''check if the trajectory will intersect the collision region'''

        v = np.subtract(self.position, position_1)
        u = np.subtract(position_2, position_1)
        s = np.dot(v, u) / np.sqrt(np.dot(u, u))

        if s < 0:
            c = position_1
        elif s > 1:
            c = position_2
        else:
            c = position_1 + s * u

        n = np.subtract(self.position, c)
        d = np.sqrt(np.dot(n, n))

        if d < self.collision_radius:
            return True
        return False


    def compute_force(self, position):
        '''compute the field force of this instance on given position'''

        d = self.distance(position)
        if d < self.field_radius:
            return np.array([0.0, 0.0])

        g = np.exp(-d * d)

        dir = self.position - position
        dir = dir / np.sqrt(np.dot(dir, dir))
        return self.multiplier * g * dir


    def render(self, figure, ax):
        ax.scatter(self.position, color="black",  s=5)
        ax.scatter(self.position, color="orange", s=20, alpha=0.1)


class Line_Obstacle:
    def __init__(self, properties):
        self.is_horizontal = properties["horizontal"]
        self.start = np.array([properties["start"]["x"], properties["start"]["y"]])
        self.end   = np.array([properties[  "end"]["x"], properties[  "end"]["y"]])
        self.collision_radius = 0.1 # collision radius of obstacles
        self.field_radius  = 0.5 # field radius of obstacles


    def distance(self, position):
        '''compute the minimum distance from position to any point on line obstacle'''

        v = np.subtract(position, self.start)
        u = np.subtract(self.end, self.start)
        s = np.dot(v, u) # no need to divide by length of line segment due to unit length segment

        if s < 0:
            c = self.start
        elif s > 1:
            c = self.end
        else:
            c = self.start + s * u

        n = np.subtract(position, c)
        return np.sqrt(np.dot(n, n))


    def is_in_collision(self, position):
        '''check if position is in collision with this obstacle'''

        if self.distance(position) < self.collision_radius:
            return True
        return False


    def is_intersecting(self, position_1, position_2):
        '''
        check if the trajectory from position 1 to 2 intersect
        this line obstacle.
        Implementation ref:
            https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        '''

        # helper functions
        def orientation(p, q, r):
            val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
            if (val > 0): # Clockwise orientation
                return 1
            elif (val < 0): # Counterclockwise orientation
                return 2
            else: # Collinear orientation
                return 0

        def onSegment(p, q, r):
            if ((q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and
                (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
                return True
            return False

        if self.is_horizontal:
            extra = np.array([self.collision_radius, 0])
        else:
            extra = np.array([0, self.collision_radius])

        p1 = self.start - extra
        q1 = self.end   + extra
        p2 = position_1
        q2 = position_2

        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        # General case
        if ((o1 != o2) and (o3 != o4)):
            return True

        # Special Cases
        # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
        if ((o1 == 0) and onSegment(p1, p2, q1)):
            return True
        # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
        if ((o2 == 0) and onSegment(p1, q2, q1)):
            return True
        # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
        if ((o3 == 0) and onSegment(p2, p1, q2)):
            return True
        # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
        if ((o4 == 0) and onSegment(p2, q1, q2)):
            return True
        # If none of the cases
        return False


    def compute_force(self, position):
        '''compute the field force of this obstacle on given position'''

        d = self.distance(position)
        if d < self.field_radius:
            return np.array([0.0, 0.0])

        g = np.exp(-d * d)
        if self.is_horizontal:
            return np.array([0.0, g])
        else:
            return np.array([g, 0.0])


    def render(self, figure, ax):
        '''draw the obstacle on figure'''

        ax.plot([self.start[0], self.end[0]], [self.start[1], self.end[1]], color="black",  linewidth=2)
        ax.plot([self.start[0], self.end[0]], [self.start[1], self.end[1]], color="orange", linewidth=10, alpha=0.1)


class Map:
    def __init__(self, map_json):
        '''load start and goal location as well as a list of obstacles'''
        self.start = None
        self.goal = None
        self.obstacles = []

        with open(map_json, 'r') as f:
            map_dict = simplejson.load(f)

        self.grid_size = map_dict["grid_size"]
        self.start = np.array([map_dict["start"]["x"], map_dict["start"]["y"]])
        self.goal  = np.array([map_dict[ "goal"]["x"], map_dict[ "goal"]["y"]])

        for obstacles in map_dict["line_obstacles"]:
            self.obstacles.append(Line_Obstacle(obstacles))

        print("[IFNO]: Map initialized.")


    def is_in_bound(self, position):
        '''check whether position is still in the map area'''

        if position[0] < 0 or position[0] > self.grid_size:
            return False
        if position[1] < 0 or position[1] > self.grid_size:
            return False
        return True


    def is_in_collision(self, position):
        '''check whether given position is in collision with obstables'''

        for obstacle in self.obstacles:
            if obstacle.is_in_collision(position):
                return True
        return False


    def is_reachable(self, start_position, end_position):
        '''
        check whether end position can be reached from start position
        in a straight line fashion
        '''

        for obstacle in self.obstacles:
            if obstacle.is_intersecting(start_position, end_position):
                return False
        return True


    def compute_goal_force(self, position):
        '''return unit vector from given position to goal position'''

        dir = np.subtract(self.goal, position)
        return np.divide(dir, np.sqrt(np.dot(dir, dir)))


    def compute_obstacles_force(self, position):
        '''
        find all obstacles that has an effect on given position and
        compute the net gradient
        '''

        force = np.array([0.0, 0.0])

        for obstacle in self.obstacles:
            force += obstacle.compute_force(position)

        return force


    def compute_net_force(self, position, alpha=1.0, beta=1.0):
        '''
        combine goal force and obstacles' force
        net_force = alpha * goal_force + beta * obstacles_force
        '''

        force  = self.compute_goal_force(position)
        force += self.compute_obstacles_force(position)
        return force


    def render(self):
        '''draw the map'''

        figure, ax = plt.subplots(figsize=(self.grid_size, self.grid_size))
        ax.scatter(0.5, 0.5, s=5, color="green", marker="o")
        ax.scatter(self.grid_size - 0.5, self.grid_size - 0.5, s=5, color="red", marker="o")

        for obstacle in self.obstacles:
            obstacle.render(figure, ax)

        return figure, ax