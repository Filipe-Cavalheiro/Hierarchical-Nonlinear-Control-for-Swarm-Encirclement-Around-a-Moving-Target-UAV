# Hierarchical Nonlinear Control for Swarm Encirclement Around a Moving Target UAV

This project contains two simulation implementations (Matlab, Python), one real world experiment (Crazy Flies) as well as the presentation files(manim):

Both simulation repositories contain (nearly) identical code and simulate the behavior and control of multiple UAVs performing encirclement maneuvers around a moving target. The MATLAB version currently provides more stable and reliable simulation results, especially the plots.

The real world experiment uses crazy flies and the light house tracking equipment.

For detailed information about the implementation, configuration, and usage instructions, please refer to the respective `README.md` files located inside each repository folder. These contain the complete documentation specific to each version.

## Citation

If you use this repository in your research, please cite the associated publication.

```bibtex
@INPROCEEDINGS{11771846,
  author={Cavalheiro, Filipe and Guerreiro, Bruno J.},
  booktitle={2026 17th APCA International Conference on Automatic Control and Soft Computing (CONTROLO)}, 
  title={Hierarchical Nonlinear Control for Swarm Encirclement Around a Moving Target UAV}, 
  year={2026},
  volume={},
  number={},
  pages={1-6},
  keywords={Equations;Stars;Autonomous aerial vehicles;Tagging;US Department of Transportation;Drones;Simulation;Trajectory;Convergence;Modeling;UAV;encirclement;swarm;geometric control;Lyapunov stability;two-sphere},
  doi={10.1109/CONTROLO67943.2026.11771846}}
```

## License

This work uses the AGPL license more information about the same can be viewed at `LICENCSE`.

## Future Work

Potential extensions include:

- Decentralized estimation and assignment
- Obstacle avoidance
- Time-varying communication networks
- Hardware-in-the-loop simulation
- ROS 2 integration
- PX4 implementation
