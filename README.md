<!--
 * @Author: piluohong 1912694135@qq.com
 * @Date: 2024-03-04 16:27:35
 * @LastEditors: piluohong 1912694135@qq.com
 * @LastEditTime: 2024-04-10 09:31:21
 * @FilePath: /slam/hhh_ws/src/hc-lio/README.md
 * @Description: 3D localization and mapping of multi-agricultural scenes via a hierarchically-coupled LiDAR-Inertial Odometry
-->
# hc-lio
Repository for 3D localization and mapping of multi-agricultural scenes via a hierarchically-coupled LiDAR-Inertial Odometry

The repository only serves as a personal record, code will come soon.

Experiment in farming field:
![Alt text](figures/result.png)

Dataset Table
![alt text](figures/dataset_table.png)

Our datasets: https://drive.google.com/drive/folders/1-SLxUejiFGY_PzGn1oLpMKWUoBMMOyx5 (now closed this link for some reseaons)

Additinal test： hku_main_building
![alt text](figures/hku_main_building.png)
![alt text](figures/hku_main_building_VGICP.png)
![alt text](figures/hku_main_building_traj_VGICP.png)


TODO:
add gravity factor;
add submap management based point-based or voxel-based;
...

Acknowledgements:
```
@article{chen2022dlio,
  title={Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction},
  author={Chen, Kenny and Nemiroff, Ryan and Lopez, Brett T},
  journal={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  pages={3983-3989},
  doi={10.1109/ICRA48891.2023.10160508}
}

@Booklet{EasyChair:2703,
  author = {Kenji Koide and Masashi Yokozuka and Shuji Oishi and Atsuhiko Banno},
  title = {Voxelized GICP for Fast and Accurate 3D Point Cloud Registration},
  howpublished = {EasyChair Preprint no. 2703},

  year = {EasyChair, 2020}}

@book{factor_graphs_for_robot_perception,
    author={Frank Dellaert and Michael Kaess},
    year={2017},
    title={Factor Graphs for Robot Perception},
    publisher={Foundations and Trends in Robotics, Vol. 6},
    url={http://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf}
}
