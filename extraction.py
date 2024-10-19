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
        self.enabled: bool = False # whether the extractor is enabled

    @abstractmethod
    def pause_extraction(self):
        '''
        Pauses the extraction mode. Primarily used for corners
        '''

    @abstractmethod
    def unpause_extraction(self):
        '''
        Unpauses the extraction mode. Primarily used for corners
        '''

    def disable_extraction(self):
        '''
        Disables the extraction mode. A second-level control which is stricter than pausing
        '''
        self.enabled = False

    def enable_extraction(self):
        '''
        Enables the extraction mode. A second-level control which is stricter than pausing
        '''
        self.enabled = True

    def reset_extraction(self):
        '''
        Resets the extraction cycle to the start
        '''
        pass # by default does nothing

    def attach_agents(self, robot: Robot):
        '''
        Register the robot agent to the extractor

        Parameters:
            robot (Robot): the robot that will be used to send commands
        '''
        self.robot = robot

    @abstractmethod
    def spin(self):
        '''
        A function to call extraction logic every loop iteration.
        '''

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
        self.paused = False

        self.extraction_start_time = None

    def pause_extraction(self):
        if self.enabled:
            self.extraction_pause_time = time.time()
            self.paused = True

    def unpause_extraction(self):
        if self.enabled:
            if self.extraction_pause_time is not None:
                # offsets the extraction time according to time already paid
                self.old_extraction_time = time.time() - (self.extraction_pause_time - self.old_extraction_time)
                self.extraction_pause_time = None
                self.paused = False

    def enable_extraction(self):
        self.reset_extraction()
        return super().enable_extraction()

    def disable_extraction(self):
        return super().disable_extraction()

    def spin(self):
        if self.enabled and not self.paused:
            if self.extraction_start_time is None: # we haven't started extraction
                if time.time() - self.old_extraction_time > self.EXTRACTION_INTERVAL:
                    self.robot.send_control_action(0, 0, do_print=False)
                    self.robot.send_control_command("command=9")
                    self.extraction_start_time = time.time()
            else:
                if time.time() - self.extraction_start_time > self.EXTRACTION_DURATION:
                    self.old_extraction_time = time.time()
                    self.extraction_start_time = None

    def reset_extraction(self):
        self.old_extraction_time = time.time()

class ExtractionContinuous(ExtractionModes):
    '''
    Continuously runs the extraction motor.
    '''
    def __init__(self):
        super().__init__()

    def pause_extraction(self):
        # send the stop command if enabled
        if self.enabled:
            self.robot.send_control_command("command=8")

    def unpause_extraction(self):
        # send the start command
        if self.enabled:
            self.robot.send_control_command("command=7")

    def disable_extraction(self):
        self.pause_extraction()
        super().disable_extraction()

    def enable_extraction(self):
        super().enable_extraction()
        self.unpause_extraction()

    def spin(self):
        return

class ExtractionNone(ExtractionModes):
    '''
    Does not run the extraction motor.
    '''
    def __init__(self):
        return

    def pause_extraction(self):
        return

    def unpause_extraction(self):
        return

    def spin(self):
        return
