'''
Holds the different types of extractions which can be performed.

Classes:
    ExtractionModes(): an abstract class representing any type of extraction.
        ExtractionPeriodic(): runs the extraction procedure periodically
        ExtractionContinuous(): always runs extraction
        ExtractionNone(): never runs extraction
'''

from abc import ABC, abstractmethod
from robot import *
import time

class ExtractionModes(ABC):
    '''
    Represents any type of extraction mode.
    '''

    def __init__(self):
        self.robot: Robot = None # the robot linked to the extractor

    @abstractmethod
    def pause_extraction(self):
        '''
        Pauses the extraction mode
        '''

    @abstractmethod
    def unpause_extraction(self):
        '''
        Unpauses the extraction mode
        '''

    def reset_extraction(self):
        '''
        Resets the extraction cycle to the start
        '''
        self.pause_extraction() # by default just pauses

    def attach_agents(self, robot: Robot):
        '''
        Register the robot agent to the extractor

        Parameters:
            robot (Robot): the robot that will be used to send commands
        '''
        self.robot = robot

class ExtractionPeriodic(ExtractionModes):
    '''
    Runs the extraction procedure (one full revolution) every period.
    '''
    def __init__(self, duration: float, interval: float):
        '''
        Parameters:
            duration (float): how long the extraction procedure takes [s]
            interval (float): the time between the end of one procedure and the start of the other [s]
        '''
        super().__init__()

        # sets the hyperparameters
        self.EXTRACTION_DURATION = duration
        self.EXTRACTION_INTERVAL = interval

        # initialise the extraction parameters
        self.old_extraction_time = time.time()
        self.extraction_pause_time = None
    
    def pause_extraction(self):
        self.extraction_pause_time = time.time()
    
    def unpause_extraction(self):
        if self.extraction_pause_time is not None:
            # offsets the extraction time according to time already paid
            self.old_extraction_time = time.time() - (self.extraction_pause_time - self.old_extraction_time)
            self.extraction_pause_time = None
    
    def reset_extraction(self):
        self.old_extraction_time = time.time()
        super().reset_extraction()

class ExtractionContinuous(ExtractionModes):
    '''
    Continuously runs the extraction motor.
    '''
    def __init__(self):
        super().__init__()

    def pause_extraction(self):
        # send the stop command
        self.robot.send_control_command("command=8")
    
    def unpause_extraction(self):
        # send the start command
        self.robot.send_control_command("command=7")

class ExtractionNone(ExtractionModes):
    '''
    Does not run the extraction motor.
    '''
    def __init__(self):
        pass

    def pause_extraction(self):
        pass

    def unpause_extraction(self):
        pass