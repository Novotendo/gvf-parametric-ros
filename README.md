# GVF Parametric ROS

This package is a **singularity-free guiding vector field for robot navigation**. It varies from other vector-field guided path-following (VF-PF) algorithms in the next ways:

* It is  a novel method to transform self-intersected or simple closed desired paths to non-self-intersected and unbounded counterparts in a higher dimensional space.

* This approach is a combining extension of both conventional VF-PF and trajectory tracking algorithms.

* Global convergence to the desired paths, which could be even self-intersected, is rigorously guaranteed.

# Paper

If you use this software in a scientific publication, please cite the following paper:

W. Yao, H. G. de Marina, B. Lin and M. Cao, **"Singularity-Free Guiding Vector Field for Robot Navigation"** in IEEE Transactions on Robotics, vol. 37, no. 4, pp. 1206-1221, Aug. 2021, doi: 10.1109/TRO.2020.3043690.

```latex
@ARTICLE{Yao2021gvfparametric,
  author={Yao, Weijia and de Marina, Héctor Garcia and Lin, Bohuan and Cao, Ming},
  journal={IEEE Transactions on Robotics}, 
  title={Singularity-Free Guiding Vector Field for Robot Navigation}, 
  year={2021},
  volume={37},
  number={4},
  pages={1206-1221},
  doi={10.1109/TRO.2020.3043690}}
```

# Credits
The spline curve interpolation library used was mainly adapted from the contribution of Tino Kluge (https://kluge.in-chemnitz.de/opensource/spline/).
