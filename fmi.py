import fmpy
import fmpy.fmi2
import time
import numpy as np
import matplotlib.pyplot as plt

from robot import Robot, wrapToPi
from localisation import Localisation

class RobotSim(Robot, Localisation):
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
        self.step_size = 0.05
        self.last_sim_time = 0.0

        self.results = []
    
    def dump(self):
        fmpy.dump(self.model)
    
    def get_position(self, *args):
        while time.time() - self.last_sim_time >= self.step_size:
            self.step()

        x, y, theta =  self.fmu.getReal([self.vrs["x"], self.vrs["y"], self.vrs["theta"]])
        _ = 0
        theta = wrapToPi(theta)
        return (x, _, y, _, _, _), (theta, _, _, _, _, _)
    
    def step(self):
        self.fmu.doStep(currentCommunicationPoint=self.time, communicationStepSize=self.step_size)
        self.time += self.step_size
        self.last_sim_time = time.time()

        self.results.append((self.time, *self.fmu.getReal([self.vrs["x"], self.vrs["y"], self.vrs["theta"]])))

    def deinit(self):
        self.plot_results()
        self.fmu.terminate()
        self.fmu.freeInstance()
    
    def send_control_action(self, v: float, omega: float, do_print=False):
        if do_print:
            print(f"v: {v}, omega: {omega}")
        
        self.fmu.setReal([self.vrs["v"], self.vrs["omega"]], [v, omega])
    
    def plot_results(self):
        fmpy.util.plot_result(np.array(self.results, dtype=np.dtype([('time', np.float64), ('x', np.float64),  ('y', np.float64),  ('theta', np.float64)])))

        data = np.array(self.results)
        plt.plot(data[:, 1], data[:, 2])
        plt.xlim((0, 2000))
        plt.ylim((0, 2000))
        plt.show()


pass