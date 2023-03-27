#!/usr/bin/env python
"""
| File: 1_px4_single_vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation with a single vehicle, controlled using the MAVLink control backend.
"""

# Imports to start Isaac Sim from this script
import carb
from omni.isaac.kit import SimulationApp

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self, simulation_app: SimulationApp, world: str, world_settings=None):  
        """
        Method that initializes the PegasusApp and setup the simulation environment
        
        Args:
            simulation_app (SimulationApp): The simulation app that will be used to run the simulation.
            world (str): A string with either the name of a standard world or the path to a custom USD world.
            world_settings (dict, optional): A dictionary with the settings for the world. Defaults to None.
        """

        # Save a simulation app reference
        self.simulation_app: SimulationApp = simulation_app

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings) if world_settings is None else World(**world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA or a custom world
        if world in SIMULATION_ENVIRONMENTS:
            self.pg.load_environment(SIMULATION_ENVIRONMENTS[world])
        else:
            self.pg.load_environment(world)

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def start(self):

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while self.simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        self.simulation_app.close()