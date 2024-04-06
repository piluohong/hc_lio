<!--
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2024-03-04 16:27:35
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2024-04-06 12:34:10
 * @FilePath: /slam/hhh_ws/src/hc-lio/README.md
 * @Description: 3D localization and mapping of multi-agricultural scenes via a hierarchically-coupled LiDAR-Inertial Odometry
-->
# hc-lio
Repository for 3D localization and mapping of multi-agricultural scenes via a hierarchically-coupled LiDAR-Inertial Odometry


Experiment in farming field:
![Alt text](figures/result.png)

Our datasets: https://drive.google.com/drive/folders/1-SLxUejiFGY_PzGn1oLpMKWUoBMMOyx5?usp=drive_link

Dataset Table
![alt text](figures/dataset_table.png)

Additinal test： hku_main_building
![alt text](figures/hku_main_building.png)
![alt text](figures/hku_main_building_VGICP.png)
![alt text](figures/hku_main_building_traj_VGICP.png)


TODO:
add gravity factor;
add submap management;
...

Acknowledgment:
```
@article{chen2022dlio,
  title={Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction},
  author={Chen, Kenny and Nemiroff, Ryan and Lopez, Brett T},
  journal={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  pages={3983-3989},
  doi={10.1109/ICRA48891.2023.10160508}
}