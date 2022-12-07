# Kimarite
This project proposes an autonomous vehicle model for the use in microscopic traffic simulations. To evaluate the models 
behaviour first tests have been carried out interfacing the autonomous agent with the microscopic traffic simulation SUMO.


## Project Structure

This project features the following key elements:
* An Agent model to be used in various scenarios
* A Controller for the Agent making it autonomous
* Testing Data from a Bike Simulator Study carried out at TUM
* An Interface to connect the autonomous agent to SUMO
* Different Plotting Functionality to create appealing results
* Different Path Planning Algorithms such as RRT* and A*


### The Agent
![Showing the agent model with different controllers fit](figures/sensor_layout.png)

The Agent can be fitted with a variety of sensors, whose implementation is based on sensors used for real autonomous vehicles.
Currently, a LiDAR sensor model and a simple Camera is supported. Both perception devices can be adjusted using different
parameters and placed at an arbitrary position on the vehicle.


### The Controller
![The implemented Controller](figures/controller.png)

The Controller combines the main components of an autonomous vehicle: Perception, Planning, Control, and Communication.
In addition to these compulsory elements, a data monitor and plotting functionality has been added in order to gather and 
evaluate simulation results. 

### Interfacing with SUMO
![Interface with SUMO](figures/sumo_parallel.PNG)

![Showing the agent within the simulation](figures/g2366.png)

The interface with SUMO allows the simultaneous simulation within SUMO an Python's Matplotlib. The Simulation within Matplotlib
can be adjusted and extended to ones liking, showing all outputs of the different controller output.

### Results
![Results](figures/Figure_2.png)

The results show:
* The planned itinerary of the agent given by a combination of lanes and junctions of the respective SUMO 
network generated by an A* Path Planner and highlighted in yellow (top left)
* The resulting trajectory using colour coded beacons indicating the agent's speed (top right)
* The border data gathered by the sensor as red xs (bottom left)
* The sensed borders based on the sensor data (left=red, right=green) and the resulting desired path (purple) (bottom right)

### More Information
More Information can be found in my thesis, where everythigng is explained in more detail. For questions don't hesitate to send a message.
