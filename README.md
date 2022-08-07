# Drone swarm simulation platform

The objective of this function is to provide a simulation platform in Python for drone swarming. This function is inspired by [Wil Selby](https://github.com/wilselby/MatlabQuadSimAP), for single drone coded in MATLAB.  

The simulations of distributed localization algorithm and  obstacle avoidance within Melbourne city using buildings data were later adopted into this function.

# Note

The paramters, such as PID parameter, drone parameter, sensor noise and etc., can be modified in `quad_variables.py`. The number of drone can be modified in `main_simple.py`, where the initial positions and desired positions should be given accordingly.


# Demo

There are 8 drones in this demo. Their initial position are random and try to attain the desired positions via formation control and collision avoidance algorithm.

![](/Figures/demo.gif)

# Reference

Riccardo Falconi, Lorenzo Sabattini, Cristian Secchi, Cesare Fantuzzi and Claudio Melchiorri. [Edge-weighted consensus-based formation control strategy with collision avoidance.](https://www.cambridge.org/core/journals/robotica/article/edgeweighted-consensusbased-formation-control-strategy-with-collision-avoidance/5F6F1839920DF93E2246FB80B41E46CE)
