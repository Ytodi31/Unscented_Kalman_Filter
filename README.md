# Unscented Kalman Filter

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<p align="center">
<img src="/data/output.gif" width="650" height="250" />
</p>

## Overview
The project uses sensor information from LiDAR and RADAR and predicts the position and velocities of vehicles around the ego vehicle, replicating the highway driving scenario. The project uses Unscented Kalman Filter for prediction and sensor fusion.



## Build and Run
`git clone https://github.com/Ytodi31/Unscented_Kalman_Filter`
`mkdir build` \
`cd build` \
`cmake ..` \
`make` \
`./ukf_highway`

## Results
- The output of the Unscented Kalman FIlter is shown in the video. The accuracy of the implemented Unscented Kalman Filter is calculated using Root Mean Square Error between the estimation and ground truth. It can be observed that the RMSE values are low and seem to converge to zero with increasing measurements. \
- The estimations using LiDAR measurements are shown in green and the estimations using RADAR measurements are shown in purple. The magnitude of these estimations are displayed in the output and the direction is shown by arrows.
- It is observed that the RMSE values increase if measurements are taken from only one sensor, and the RMSE values converge faster is sensor fusion is done with LiDAR and RADAR measurements.
