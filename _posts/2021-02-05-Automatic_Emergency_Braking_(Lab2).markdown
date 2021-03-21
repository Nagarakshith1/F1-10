# Automatic Emergency Braking

Automatic Emergency(AEB) is a safety feature used to stop the car if the time to collision is less than a certain threshold. \
The implementaion details are as follows.
1) The LIDAR scans obtained are processed in order to find the minimum time to collision.
2) The time to collision can be expressed as: \
![equation](https://latex.codecogs.com/gif.latex?ttc&space;=\frac{r}{max(\dot{r},0))
3) The above calculation can be intrepreted geometrically as the division of the obstacle distance of a LIDAR ray by the projection of velocity of the car onto that LIDAR ray.
4) The negative value of this divison is equivalent to infinite time to collision. If the minimum time thus obtained is lesser than a threshold then the brakes have to be applied.
5) The threshold can be calculated approximately by considering the maximum velocity and deceleration of the car.
6) However since the car is not circular in shape the distance to the obstacle after stopping is not the same when driven backwrads and forwards. This can be eliminated by subtracting the distances from the center of the LIDAR to the car edges along all the diffrent LIDAR rays. All these distances can be precomputed as they are fixed.
7) These distances can be computed by firstly considering the concurrent lines oriented with the resolution of the LIDAR scan angles. The car edges are represented as lines and can be solved for the intersection points with those radial lines. These intersection points need to be translated to the LIDAR frame and finally euclidian distances from the origin can be calculated.
## Rviz View
*Click on to watch the video*
>
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/5TWWysJexSs/0.jpg)](https://www.youtube.com/watch?v=5TWWysJexSs)

[Code](https://github.com/Nagarakshith1/F1Tenth/tree/gh-pages/lab2)\
[Assignment](https://f1tenth.org/learn.html)