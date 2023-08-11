<h1 align="center">Flocking Algorithm and Formation Control</h1>

<h6 align="center"><small>MATLAB implementations of Formation Control Algorithm.</small></h6>

<p align="center"><b>#multi-agent system &emsp; #formation control  &emsp; #Communication-aware <br/> #behavior-based  &emsp; #jamming area  &emsp; #obstacle avoidance</b></p>

This paper presents a formation control strategy for a swarm of unmanned aerial vehicles (UAVs) that navigate towards a destination while avoiding a jamming area. The proposed approach utilizes a gradient controller for formation control to maximize the communication quality within the swarm. A movement controller is used to go to a destination and avoid a jamming area without prior knowledge of its existence. A simulation is conducted to prove the effectiveness of this approach. The result highlights an efficient navigation strategy while maintaining sufficient communication quality for a multi-agent system.

<div align="center">

<h2>Paper</h2>

<div>
      <a href="https://github.com/sjpeccoud/Formation-Control/tree/main/lib/Preprint.pdf">
        <img src="/lib/README.assets/Preprint.png" width="350"/>
      </a>
  
  [![](https://img.shields.io/badge/View%20More-282c34?style=for-the-badge&logoColor=white)](https://github.com/sjpeccoud/Formation-Control/tree/main/lib/Preprint.pdf)
</div>

<h2>Presentation</h2>

<div>
      <a href="https://github.com/sjpeccoud/Formation-Control/tree/main/lib/Presentation.pdf">
        <img src="/lib/README.assets/Presentation.png" width="350"/>
      </a>
  
  [![](https://img.shields.io/badge/View%20More-282c34?style=for-the-badge&logoColor=white)](https://github.com/sjpeccoud/Formation-Control/tree/main/lib/Presentation.pdf)
</div>

<h2>Simulation</h2>

<h3>Scenario 1: Move to Destination</h3>

<table>
  <tr>
    <th>Initial</th>
    <th>Halfway</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Swarm2Dest_1.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Swarm2Dest_2.png" width="500" /></td>
  </tr>
  <tr>
    <th>Final</th>
    <th>Trajectory</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Swarm2Dest_3.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Swarm2Dest_trace.png" width="500" /></td>
  </tr>
</table>

<h3>Scenario 2: Avoid Jamming Area</h3>

<table>
  <tr>
    <th>Initial</th>
    <th>Halfway</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Avoid_No_Follow_1.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Avoid_No_Follow_2.png" width="500" /></td>
  </tr>
  <tr>
    <th>Final</th>
    <th>Trajectory</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Avoid_No_Follow_3.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Avoid_No_Follow_trace.png" width="500" /></td>
  </tr>
</table>

<h3>Scenario 3: Follow Jamming Area Edge</h3>

<table>
  <tr>
    <th>Initial</th>
    <th>Halfway</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Wall_Follow_1.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Wall_Follow_2.png" width="500" /></td>
  </tr>
  <tr>
    <th>Final</th>
    <th>Trajectory</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Wall_Follow_3.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Wall_Follow_trace.png" width="500" /></td>
  </tr>
</table>

<h2>Final Simulation</h2>

<table>
  <tr>
    <th>Initial</th>
    <th>Halfway</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_1.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_2.png" width="500" /></td>
  </tr>
  <tr>
    <th>Final</th>
    <th>Trajectory</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_3.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_4.png" width="500" /></td>
  </tr>
  <tr>
    <th>Initial</th>
    <th>Halfway</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_5.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_6.png" width="500" /></td>
  </tr>
  <tr>
    <th>Final</th>
    <th>Trajectory</th>
  </tr>
  <tr>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_7.png" width="500" /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Simulation_trace.png" width="500" /></td>
  </tr>
</table>

</div>

<!-- <h2 align="center">BibTeX Citation</h2>

```
@John Doe{John Doe,
  author={John Doe},
  booktitle={John Doe}, 
  title={John Doe}, 
  year={2023},
  volume={},
  number={},
  pages={},
  doi={}}
``` -->

<h2 align="center">References</h2>

[[1]](https://www.sciencedirect.com/science/article/abs/pii/S0016003215001593) H. Li, J. Peng, W. Liu, K. Gao, and Z. Huang, *“A novel communicationaware formation control strategy for dynamical multi-agent systems,”* Journal of the Franklin Institute, vol. 352, no. 9, pp. 3701–3715, 2015.

[[2]](https://www.hindawi.com/journals/mpe/2014/205759/) D. Xu, X. Zhang, Z. Zhu, C. Chen, P. Yang et al., *“Behavior-based formation control of swarm robots,”* mathematical Problems in Engineering, vol. 2014, 2014.