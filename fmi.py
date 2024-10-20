'''
Functionality for integrating the Simulink model to the pathplanning via the Functional Mockup Interface (FMI).
For more information on FMI, see https://fmi-standard.org/
'''

import fmpy
import fmpy.fmi2
import time
import numpy as np
import matplotlib.pyplot as plt

from robot import Robot
from utils import wrap_to_pi
from localisation import Localisation


class RobotSim(Robot, Localisation):
    """
    A class mocking away the robot and localisation into a simulator.
    """

    fmu_file: str  # the FMU file to reference

    def __new__(cls, filename):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotSim, cls).__new__(cls)
            cls.instance.init(filename)

        return cls.instance

    def __init__(self, filename):
        pass

    def init(self, filename: str):
        """
        Initialises the robot simulator.

        Parameters:
            filename (str): the FMU file
        """

        # read in the FMU and store characteristics
        self.model: str = filename
        self.model_description: fmpy.model_description.ModelDescription = (
            fmpy.read_model_description(self.model)
        )
        self.variables: list = self.model_description.modelVariables

        # set up a dictionary of value references
        self.vrs: dict = {}
        for variable in self.variables:
            self.vrs[variable.name] = variable.valueReference

        # intialise FMU slave
        self.fmu = fmpy.fmi2.FMU2Slave(
            guid=self.model_description.guid,
            unzipDirectory=fmpy.extract(self.model),
            modelIdentifier=self.model_description.coSimulation.modelIdentifier,
            instanceName="Simulator Instance",
        )

        # set up and initialise slave
        self.fmu.instantiate()
        self.fmu.setupExperiment(startTime=0.0)
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()

        # store time variables
        self.time: float = 0.0
        self.step_size: float = 0.05
        self.last_sim_time: float = 0.0

        # store results of simulator
        self.results: list = []

    def setup(self):
        return # no setup necessary

    def get_position(self, *args):

        # make a simulator step if it is time
        while time.time() - self.last_sim_time >= self.step_size:
            self.step()

        # get the simulated robot position
        x, y, theta = self.fmu.getReal(
            [self.vrs["x"], self.vrs["y"], self.vrs["theta"]]
        )
        _ = 0
        theta = wrap_to_pi(theta)
        return (x, _, y, _, _, _), (theta, _, _, _, _, _)

    def step(self):
        """
        Step the simulator one time step.
        """

        # execute step
        self.fmu.doStep(
            currentCommunicationPoint=self.time,
            communicationStepSize=self.step_size
        )

        # update timing parameters
        self.time += self.step_size
        self.last_sim_time = time.time()

        # save results
        self.results.append(
            (
                self.time,
                *self.fmu.getReal([self.vrs["x"],
                                   self.vrs["y"],
                                   self.vrs["theta"]]),
            )
        )

    def deinit(self):
        self.plot_results()
        self.fmu.terminate()
        self.fmu.freeInstance()

    def send_control_action(self, v: float, omega: float, do_print=False):
        if do_print:
            print(f"v: {v}, omega: {omega}")
 
        self.fmu.setReal([self.vrs["v"], self.vrs["omega"]], [v, omega])

    def send_control_command(self, command: str):
        print(f"Would send command {command}")

    def send_command(self, command):
        print(f"Would send command {command}")

    def plot_results(self):
        '''
        Plot the results of the FMU simulation
        '''
 
        # plot robot positions over time
        fmpy.util.plot_result(
            np.array(
                self.results,
                dtype=np.dtype(
                    [
                        ("time", np.float64),
                        ("x", np.float64),
                        ("y", np.float64),
                        ("theta", np.float64),
                    ]
                ),
            )
        )

        # plot 2D robot position
        data = np.array(self.results)
        plt.plot(data[:, 1], data[:, 2])
        plt.xlim((0, 2000))
        plt.ylim((0, 2000))
        plt.show()