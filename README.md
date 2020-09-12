# AMIR-SLAM: Autonomous Mobile Industrial Robot Simultaneous Localization and Mapping

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/system_overview.png?token=AF244QECXSZSGSA4QIKZMY27MW5NE" width="640">

Our aim is to design a SLAM system that is specifically for mobile manipulators and can be adopt into autonomous mobile manipulation which is the most important foundation of lots of industrial applications such as performing pick and place tasks between discrete work stations in factories. The first objective is to propose a new SLAM system leveraging the information from the AMIR such as RGB-D data, 2D laser scan data, inertial data and transformation data to get the best quality on both 2D and 3D maps among existing methods which can be adopted to an AMIR. The second objective is to adopt our SLAM system to mobile manipulation, making the mobile manipulation between discrete work stations autonomous without colliding on obstacles. In the result, our whole system based on an AMIR is shown figure below, the robot can move ubiquitously in the indoor environment and easily complete the pick and place task between discrete work stations.

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/mapping_result.png?token=AF244QDAVMPGDCUASHDBAXK7MW5S6" width="840">

## Related Publication
*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using our work for your own research:*
- R. C. Luo, S. L. Lee, Y. C. Wen and C. H. Hsu, "Modular ROS Based Autonomous Mobile Industrial Robot System for Automated Intelligent Manufacturing Applications," 2020 IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM), Boston, MA, USA, 2020, pp. 1673-1678, doi: 10.1109/AIM43001.2020.9158800.
- S. L. Lee, "Autonomous Mobile Industrial Robot with Multi-Sensor Fusion Based Simultaneous Localization and Mapping for Intelligent Service Applications," 2020 National Taiwan University Master Thesis, Taipei, Taiwan, 2020, doi: 10.6342/NTU202003847
- R. C. Luo and S. L. Lee, "Multi-Sensor Based Simultaneous Localization and Mapping System for Autonomous Mobile Industrial Robotic Applications," in IEEE Access, 2020. (Submitted)

## 1. Prerequisites

## 2. Building Project

## 3. Examples

## 4. System Overview & Algorithm Walkthrough for Tuning

<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/flow_chart_high_level.png?token=AF244QB56G7ACOJAAUJBCB27MW5VW" width="410">　<img src="https://raw.githubusercontent.com/shannon112/AMIR-SLAM/master/doc/flow_chart_low_level.png?token=AF244QC7TNT2MOWBN23J7C27MW5V2" width="410">

## License
All the packages are distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
AMIR-SLAM is licenced under Apache-2.0 License.

## Acknowledgement
AMIR-SLAM is based on Cartographer SLAM (Hess, Wolfgang, Damon Kohler, Holger Rapp, and Daniel Andor. "Real-time loop closure in 2D LIDAR SLAM.")
