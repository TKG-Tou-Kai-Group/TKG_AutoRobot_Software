class Target:
    UNKNOWN = 0
    ENEMY = 1 
    OTHER = 2 
    
    def __init__(self, input_id = 0, target_x = 0.0, target_y = 0.0, input_checked_time = 0.0):
        self.id = input_id
        self.x = target_x
        self.y = target_y
        self.checked_time = input_checked_time

        self.state = self.UNKNOWN
        self.state_checked_time = 0.0
