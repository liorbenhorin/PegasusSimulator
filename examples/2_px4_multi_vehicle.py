#!/usr/bin/env python
"""
| File: 2_px4_multi_vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation with multiple vehicles, controlled using the MAVLink control backend.
"""

# Imports to start Isaac Sim from this script
from omni.isaac.kit import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.pegasus_app import PegasusApp
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

class PX4ExampleApp(PegasusApp):
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        super().__init__(simulation_app, world="Curved Gridroom")

        # Spawn 5 vehicles with the PX4 control backend in the simulation, separated by 1.0 m along the x-axis
        for i in range(5):
            self.vehicle_factory(i, gap_x_axis=1.0)        

        # Start the simulation
        self.start()

    def vehicle_factory(self, vehicle_id: int, gap_x_axis: float):
        """Auxiliar method to create multiple multirotor vehicles

        Args:
            vehicle_id (_type_): The ID of the vehicle
        """

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        
        # Create the multirotor configuration
        mavlink_config = MavlinkBackendConfig({
            "vehicle_id": vehicle_id, 
            "px4_autolaunch": True, 
            "px4_dir": "/home/marcelo/PX4-Autopilot", 
            "px4_vehicle_model": 'iris'}
        )
        config_multirotor.backends = [MavlinkBackend(mavlink_config)]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            vehicle_id,
            [gap_x_axis * vehicle_id, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor)

def main():

    # Instantiate the template app
    pg_app = PX4ExampleApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
