class Target:
    self.UNKNOWN = 0
    self.ENEMY = 1 
    self.OTHER = 2 
    
    def __init__(self, target_x = 0.0, target_y = 0.0, input_checked_time = 0.0):
        self.x = target_x
        self.y = target_y
        self.checked_time = input_checked_time

        self.state = TARGET_STATE_UNKNOWN
        self.state_checked_time = 0.0
