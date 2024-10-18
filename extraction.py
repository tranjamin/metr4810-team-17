from abc import ABC, abstractmethod
from robot import *
import time

class ExtractionModes(ABC):
    def __init__(self):
        self.robot: Robot = None

    @abstractmethod
    def pause_extraction(self):
        pass

    @abstractmethod
    def unpause_extraction(self):
        pass

    def reset_extraction(self):
        self.pause_extraction()

    def attach_agents(self, robot: Robot):
        self.robot = robot


class ExtractionPeriodic(ExtractionModes):
    def __init__(self, duration: float, interval: float):
        super().__init__()

        self.EXTRACTION_DURATION = duration
        self.EXTRACTION_INTERVAL = interval

        self.old_extraction_time = time.time()
        self.extraction_pause_time = None
    
    def pause_extraction(self):
        self.extraction_pause_time = time.time()
    
    def unpause_extraction(self):
        if self.extraction_pause_time is not None:
            self.old_extraction_time = time.time() - (self.extraction_pause_time - self.old_extraction_time)
            self.extraction_pause_time = None
    
    def reset_extraction(self):
        self.old_extraction_time = time.time()
        super().reset_extraction()

class ExtractionContinuous(ExtractionModes):
    def __init__(self):
        super().__init__()

    def pause_extraction(self):
        self.robot.send_control_command("command=8")
    
    def unpause_extraction(self):
        self.robot.send_control_command("command=7")

class ExtractionNone(ExtractionModes):
    def __init__(self):
        pass

    def pause_extraction(self):
        pass

    def unpause_extraction(self):
        pass