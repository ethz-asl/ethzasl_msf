# Custom Sensor Update Template

This guide explains how to integrate a custom sensor into ETHZ-ASL MSF.

## Workflow

1. Define measurement vector z
2. Define covariance matrix R
3. Compute residual
4. Compute measurement Jacobian H
5. Register update handler

## Example Sensors

- Optical flow
- Barometer
- LiDAR odometry
- Magnetometer
- External VIO
