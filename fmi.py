import fmpy
import fmpy.fmi2
import time

from robot import Robot

class RobotSim(Robot):
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(RobotSim, cls).__new__(cls)
        return cls.instance
    
    def __init__(self):
        pass

    def setup(self):
        pass
    
    def init(self, filename):
        self.model = filename
        self.model_description = fmpy.read_model_description(self.model)
        self.variables = self.model_description.modelVariables

        self.vrs = {}
        for variable in self.variables:
            self.vrs[variable.name] = variable.valueReference
        
        self.fmu = fmpy.fmi2.FMU2Slave(
            guid = self.model_description.guid,
            unzipDirectory = fmpy.extract(self.model),
            modelIdentifier = self.model_description.coSimulation.modelIdentifier,
            instanceName = "instance1"
        )

        self.fmu.instantiate()
        self.fmu.setupExperiment(startTime=0.0)
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()

        self.time = 0.0
        self.step_size = 0.1
        self.last_sim_time = 0.0
    
    def dump(self):
        fmpy.dump(self.model)
    
    def get_position(self, *args):
        while time.time() - self.last_sim_time >= self.step_size:
            self.step()

        x, y, theta =  self.fmu.getReal([self.vrs["x"], self.vrs["y"], self.vrs["theta"]])
        _ = 0
        return (x, _, y, _, _, _), (theta, _, _, _, _, _)
    
    def step(self):
        self.fmu.doStep(currentCommunicationPoint=self.time, communicationStepSize=self.step_size)
        self.time += self.step_size
        self.last_sim_time = time.time()

    def deinit(self):
        self.fmu.terminate()
        self.fmu.freeInstance()
    
    def send_control_action(self, v: float, omega: float, do_print=False):
        if do_print:
            print(f"v: {v}, omega: {omega}")
        
        self.fmu.setReal([self.vrs["v"], self.vrs["omega"]], [v, omega])

pass