# Solution to MATLAB and Simulink Challenge project 230: Aggressive Maneuver Stabilization for a Minidrone
This repository contains the modelling, analysis and control design of a quadcopter system developed for our Mechatronics II assignment. It includes non-linear and linearised dynamic models, stability analysis and the design and simulation of PID-based control strategies. MATLAB/Simulink is used to compare linear vs non-linear behaviour, evaluate system stability and implement trajectory control for realistic drone operation scenarios.

[Program link](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub)

[Project description link](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Aggressive%20Maneuver%20Stabilization%20for%20a%20Minidrone)

# Project details
This repository contains a nonlinear quadcopter simulation implementing cascaded PID control with a Motor Mixing Algorithm (MMA) for position and attitude stabilisation. The model includes full translational and rotational dynamics and demonstrates trajectory tracking in 3D space using MATLAB’s numerical ODE solver.

All controllers, dynamics, and parameters are contained in a single MATLAB file, allowing the simulation to be run with one command.

# How to run
1. Open MATLAB
2. Set the current folder to the repository directory
3. Run: OneClickSimulationForMatLab
4. Watch the plots and 3D trajectory

No additional toolboxes are required.

# Video Presentation
[Click here](https://youtu.be/ICeq6gTB0ho)
  
# Reference
[1] Server Racks Online, Overhead Cable Management, 2025. [Online]. Available:
https://www.server-rack-online.com/overhead-cable-management/. Accessed: May 21,
2025.

[2] Sysracks, “How are data centers cooled?”, n.d. [Online]. Available:
https://sysracks.com/blog/how-are-data-centers-cooled/. Accessed: May 20, 2025.

[3] Server Room Environments, “Thermal camera surveys for critical data centre
systems”, n.d. [Online]. Available:
https://www.serverroomenvironments.co.uk/blog/thermal-camera-surveys-for-criticaldata-centre-systems. Accessed: May 20, 2025.

[4] Black Box, “Server cabinet cooling”, n.d. [Online]. Available:
https://www.blackbox.co.uk/gb-gb/page/25703/Resources/TechnicalResources/Black-Box-Explains/Cabinets/Server-Cabinet-Cooling. Accessed: May 20,
2025.

[5] X. Zhou, J. Liu, W. Zhang, and M. Li, “Simulation of a temperature adaptive control
strategy for an IWSE economizer in a data center,” 2014. [Online]. Available:
https://www.researchgate.net/publication/26498280_Simulation_of_a_temperature_ad
aptive_control_strategy_for_an_IWSE_economizer_in_a_data_center. Accessed: May
20, 2025.

[6] J. O. Pedro, M. Dangor, and P. J. Kala, “DiƯerential evolution-based PID control of a
quadrotor system for hovering application,” in 2016 IEEE Congress on Evolutionary
Computation (CEC), 2016, pp. 2791–2798. DOI: 10.1109/CEC.2016.7744262.

[7] M. Belkheiri, A. Rabhi, A. E. Hajjaji, and C. Pegard, “DiƯerent linearization control
techniques for a quadrotor system,” ResearchGate, 2014. DOI: 10.13140/2.1.1596.
[Online]. Available:
https://www.researchgate.net/publication/259780946_DiƯerent_linearization_control_t
echniques_for_a_quadrotor_system. Accessed: May 20, 2025.

[8] M. Okasha, J. Kralev, and M. Islam, “Design experimental comparison of PID, LQR
and MPC stabilizing controllers for Parrot Mambo mini-drone,” Aerospace, vol. 9, no. 6,
p. 298, 2022. DOI: 10.3390/aerospace9060298. [Online]. Available:
https://mdpi.com/2226-4310/9/6/298. Accessed: May 20, 2025.

[9] H. Suresh, A. Sulfikar, and V. Desai, “Hovering control of a quadcopter using linear
and nonlinear techniques,” International Journal of Modelling, Identification and
Control, 2020. [Online]. Available:
https://www.researchgate.net/publication/327431213_Hovering_control_of_a_quadco
pter_using_linear_and_nonlinear_techniques. Accessed: May 20, 2025.

[10] Swarthmore College, “Transfer function to state space,” 2024. [Online]. Available:
https://lpsa.swarthmore.edu/Representations/SysRepTransformations/TF2SS.html.
Accessed: May 21, 2024.

[11] G. Delgado-Reyes, J. S. Valdez-Martínez, P. Guevara-López, and M. A. HernándezPérez, “Hover flight improvement of a quadrotor unmanned aerial vehicle using PID
controllers with an integral eƯect based on the Riemann–Liouville fractional-order
operator: A deterministic approach,” Fractal and Fractional, vol. 8, no. 11, p. 634, 2024.
DOI: 10.3390/fractalfract8110634. [Online]. Available: https://www.mdpi.com/2504-
3110/8/11/634. Accessed: May 20, 2025.

[12] The MathWorks, Inc., “ode45 — Solve nonstiƯ diƯerential equations: Medium order
method,” 2024. [Online]. Available:
https://www.mathworks.com/help/matlab/ref/ode45.html. Accessed: May 21, 2024. 
