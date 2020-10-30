# ADAS_Functions_MATLAB

## Matlab codes for ADAS functions

# Hybrid A* Algorithm for Car Trailer System

Path optimization is not included and will be added further.

Similar path planning algorithms for a car pulling trailer are discussed by [Habrador](https://blog.habrador.com/2015/11/explaining-hybrid-star-pathfinding.html) and [Atsushi Sakai](https://github.com/AtsushiSakai/HybridAStarTrailer)

The difference is that hitch point can be set not only at the midpoint of rear axle but also at the rear point of car body, more general for a car pulling a trailer.

## Perpendicular parking
![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathPlanning/PerpendicularParkingTrailer.jpg)


<p align="center">
  <img src="https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathPlanning/PerpendicularParkingTrailer.gif" alt="animated" />
</p>

## Parallel parking
![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathPlanning/ParallelParkingTrailer.jpg)


<p align="center">
  <img src="https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathPlanning/ParallelParkingTrailer.gif" alt="animated" />
</p>

Ref:
- [Practical Search Techniques in Path Planning for Autonomous Driving](http://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)

- [Application of Hybrid A* to an Autonomous Mobile Robot for
Path Planning in Unstructured Outdoor Environments](https://pdfs.semanticscholar.org/6e00/16024b257040db590d2de352556f64f46787.pdf)


# Path Tracking for Car Trailer System

Here the modus operandi of exact input/output linearization technique is used to calculate the steering wheel angle.

## Straight Path Tracking
![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathTracking/Demo_Straight.jpg)

![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathTracking/Deviation_Straight.jpg)


## Circle Path Tracking
![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathTracking/Demo_Circle.jpg)

![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathTracking/Deviation_Circle.jpg)

![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathTracking/CircleTracking.gif)

Ref:
- Reversing the General One-Trailer System: Asymptotic Curvature Stabilization and Path Tracking

# Path Tracking For Car By Using Model Predictive Control

Two different state space equations are applied for path tracking.

Compared to steering angle, the steering angle rate as controlled signal can make system more stable. It can be found in following figures.

## Steering angle as controlled signal
![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathTracking/MPC_Car.jpg)

## Steering angle rate als controlled signal
![2](https://github.com/jingtian123qwe/ADAS_Functions_MATLAB/blob/master/Animation/PathTracking/MPC_Car_DeltaS.jpg)

Ref:
- [Obstacle Avoidance Using Adaptive Model Predictive Control](https://www.mathworks.com/help/mpc/ug/obstacle-avoidance-using-adaptive-model-predictive-control.html)



# License 

MIT


# Author
- [Junyu Zhou](https://github.com/jingtian123qwe/)