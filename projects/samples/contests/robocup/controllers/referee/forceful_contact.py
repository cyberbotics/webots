class ForcefulContact:
    def __init__(self, team, number, opponent_team, opponent_number):
        self.R1_team = team
        self.R1_number = number
        self.R2_team = opponent_team
        self.R2_number = opponent_number

    def contains(self, team, number, opponent_team, opponent_number):
        if (self.R1_team == team and self.R1_number == number and
           self.R2_team == opponent_team and self.R2_number == opponent_number):
            return True
        if (self.R2_team == team and self.R2_number == number and
           self.R1_team == opponent_team and self.R1_number == opponent_number):
            return True
        return False
