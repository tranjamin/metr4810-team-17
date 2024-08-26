Testing scripts for the Kalman filter part of the localisation subsystem.

The goal of this component is to fuse IMU data with pose estimates from the camera vision system.

The data expected is:
- IMU
    - Acceleration in robot body frame, including gravity
    - Rotational velocity
- Camera system
    - Pose estimate (x,y,z,r,p,y) in world frame