import numpy as np


class ForcefulContactMatrix:
    def __init__(self, red_team_size, blue_team_size, observation_duration, foul_duration, time_step):
        self.red_team_size = red_team_size
        self.blue_team_size = blue_team_size
        self.foul_duration = int((1000 * foul_duration) / time_step)
        self.time_window_size = int((1000 * observation_duration) / time_step)
        self.time_step = time_step
        self.matrix = np.zeros([red_team_size, blue_team_size, self.time_window_size], dtype=bool)

    def clear(self, time_count):
        index = int(time_count / self.time_step) % self.time_window_size
        for i in range(self.red_team_size):
            for j in range(self.blue_team_size):
                self.matrix[i][j][index] = False

    def clear_all(self):
        self.matrix.fill(False)

    def set_contact(self, red_number, blue_number, time_count, value=True):
        index = int(time_count / self.time_step) % self.time_window_size
        self.matrix[int(red_number) - 1][int(blue_number) - 1][index] = value

    def contact(self, red_number, blue_number, time_count):
        index = int(time_count / self.time_step) % self.time_window_size
        return self.matrix[int(red_number) - 1][int(blue_number) - 1][index]

    def long_collision(self, red_number, blue_number):
        sum = 0
        for touch in self.matrix[int(red_number) - 1][int(blue_number) - 1]:
            if touch:
                sum += 1
        return sum > self.foul_duration
