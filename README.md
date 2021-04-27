# watch-and-learn-LfD-controller
Data and simulation files for learning control from expert demonstrations

The data was collected by simulating the quadrotor in closed loop with a nonlinear controller from [1]. The files Z_all, V_all contain the full trajectories and the full inputs in (z,v)-coordinates, respectively, whereas the files Z_segment, V_segment contain only the T=2.4s segments we used to construct the learned controller. We will be updating this repository to include the simulations and code that was used for the work: "Watch and Learn: Learning to control feedback linearizable systems from expert demonstrations" by A. Sultangazin, L. Fraile, L. Pannocchi, and P. Tabuada.

The file "initLfD_sim_ref.m" is a MATLAB file that uses the learned controller to follow a reference, whereas the file "initExpert_sim_ref.m" uses the controller from [1] to follow the same reference.

[1] D. Mellinger and V. Kumar, “Minimum snap trajectory generation
and control for quadrotors,” in 2011 IEEE International Conference
on Robotics and Automation, 2011, pp. 2520–2525.
