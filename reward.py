import math
class Reward:
    
    def __init__(self, params,verbose=False, track_time=False):
        self.prev_speed = 0
        self.speed = params['speed']
        self.distance_from_center = params['distance_from_center']
        self.track_width = params['track_width']
        self.waypoints = params['waypoints']
        self.closest_waypoint = params['closest_waypoints']
        self.heading = params['heading']
        
        # Get current position
        self.x = params['x']
        self.y = params['y']



    def speed_reward(self):
        """ Weighted speed reward function 
        """
        reward = 1.0
        w1 = 4.0
        w2 = 1.0
        
        # Decrease Reward of position, if DeepRacer tries to slow downs
        if self.speed < 1.3:
          w2 = 0.3
        # Increase Reward if DeepRacer goes fast
        if self.speed > 2.5:
          w1 += 1
          
        # Calculate scaled distance from center
        distance_from_center_scaled = self.distance_from_center / (self.track_width / 2.0)
          
        # Subtract scaled distance from center from 1 to give higher reward when car is closer to center
        # Provide Weighted Reward while prioritizing speed
        reward = w1 * self.speed + w2 * (1.0 - distance_from_center_scaled)
        
        return reward
    

    def inc_speed_reward(self):     
        """Reward when speed increases."""
        reward = 0
        if (self.speed > self.prev_speed) and (self.prev_speed > 0):
            reward += 10
        self.prev_speed = self.speed 
        return reward
    
    
    # Now lets use future points to reward our agent

    # x1, y1 coordinate of starting point
    # x2, y2 coordinate of ending point
    # distance: distance between points of lines
    def get_line_points(self, x1, y1, x2, y2, distance=0.1):
        """A function that creates new points between given two points."""
        dx = x2 - x1
        dy = y2 - y1
        line_length = math.sqrt(dx ** 2 + dy ** 2)
        num_points = int(line_length / distance) + 1
        x_steps = dx / (num_points - 1)
        y_steps = dy / (num_points - 1)
        line_points = [(x1 + i * x_steps, y1 + i * y_steps) for i in range(num_points)]
        return line_points

    def find_next_three_waypoints(self):
        next_points = (list(range(self.closest_waypoint[1], self.closest_waypoint[1] + 3)))
        for i in range(len(next_points)):
            if next_points[i] > len(self.waypoints):
                next_points[i] -= len(self.waypoints)
        return next_points
    
    def angle_between_points(self, first_point, x, third_point):
        """Calculates the angle between two line segments formed by three points."""
        first_dx = first_point[0] - x
        first_dy = first_point[1] - 0
        third_dx = third_point[0] - x
        third_dy = third_point[1] - 0
        angle = math.atan2(third_dy, third_dx) - math.atan2(first_dy, first_dx)
        return math.degrees(angle)



    def opt_path_reward(self):
        reward = 1


        next_points = self.find_next_three_waypoints()

        # Get Destination coordinates
        x_forward = self.waypoints[next_points[2]][0]
        y_forward = self.waypoints[next_points[2]][1]

        optimal_path = self.get_line_points(self.x, self.y, x_forward, y_forward)


        # Calculate reward for alignment with optimal steering direction
        optimal_heading = math.degrees(math.atan2(y_forward - self.y, x_forward - self.x))
        heading_diff = abs(optimal_heading - self.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        reward_alignment = math.cos(math.radians(heading_diff))

        return reward + reward_alignment

    def opt_speed_on_curve(self):
        
        next_points = self.find_next_three_waypoints()

        # Calculate curvature
        first_point = self.waypoints[next_points[0]]
        third_point = self.waypoints[next_points[2]]
        curvature = self.angle_between_points(first_point, self.x, third_point)

        # Optimal speed based on curvature
        min_speed, max_speed = 1, 4
        # Changed to continuous function for optimal speed calculation
        optimal_speed = max_speed - (curvature / 180) * (max_speed - min_speed)

        # Calculate reward for speed
        speed_diff = abs(self.speed - optimal_speed)
        reward_speed_curv = math.exp(-0.5 * speed_diff)

        return reward_speed_curv


def reward_function(params):
    # creating reward object
    reward_obj = Reward(params)

    reward = reward_obj.speed_reward()
    reward += reward_obj.inc_speed_reward()
    reward += reward_obj.opt_path_reward()
    reward += reward_obj.opt_speed_on_curve()

    return float(reward)
