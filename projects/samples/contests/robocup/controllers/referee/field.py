class Field:
    def __init__(self, size):
        self.size = size
        self.size_y = 3 if size == 'kid' else 4.5
        self.size_x = 4.5 if size == 'kid' else 7
        self.penalty_mark_x = 3 if size == 'kid' else 4.9
        self.goal_area_length = 1
        self.goal_area_width = 3 if size == 'kid' else 4
        self.goal_height = 1.2 if size == 'kid' else 1.8
        self.penalty_area_length = 2 if size == 'kid' else 3
        self.penalty_area_width = 5 if size == 'kid' else 6
        self.circle_radius = 0.75 if size == 'kid' else 1.5
        self.penalty_offset = 0.6 if size == 'kid' else 1
        self.opponent_distance_to_ball = 0.75 if size == 'kid' else 1.5
        self.ball_vincity = 0.75 if size == 'kid' else 1.5
        self.robot_radius = 0.3 if size == 'kid' else 0.5
        self.place_ball_safety_dist = 0.5 if size == 'kid' else 1.0
        self.turf_depth = 0.01
        self.border_strip_width = 1
        self.line_width = 0.05
        self.line_half_width = self.line_width / 2

    def point_inside(self, point, include_turf=False, include_border_line=True):
        if point[2] > self.turf_depth:  # in the air
            return False
        x = self.size_x + (self.border_strip_width if include_turf else 0)
        y = self.size_y + (self.border_strip_width if include_turf else 0)
        if not include_border_line:
            x -= self.line_width
            y -= self.line_width
        if point[0] > x or point[0] < -x or point[1] > y or point[1] < -y:
            return False
        return True

    def circle_fully_inside_goal_area(self, point, radius):
        return (abs(point[0]) - radius > self.size_x - self.goal_area_length and
                abs(point[0]) + radius < self.size_x and
                abs(point[1]) + radius < self.goal_area_width / 2)

    def circle_fully_inside_penalty_area(self, point, radius):
        return (abs(point[0]) - radius > self.size_x - self.penalty_area_length and
                abs(point[0]) + radius < self.size_x and
                abs(point[1]) + radius < self.penalty_area_width / 2)
