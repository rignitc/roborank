# Differential Drive Odometry

## Approach
In this policy, the pose (position and orientation) of a differential drive robot is estimated using the wheel encoders and the gyroscope. By calculating the displacement of each wheel from the wheel encoder readings and averaging it for translation estimation, a more precise heading estimation can be achieved through integration of the gyroscope information. The main principle is that between two discrete time points, it can be assumed that the robot travels in a straight line, the distance of which is estimated through the wheel encoders and the heading from the gyroscope. Subsequently, based on these estimated movements, the global pose of the robot is updated.

## Major Equations
The distances traveled by the left and right wheels ($\Delta d_L$, $\Delta d_R$) are derived from encoder changes $\Delta N$:
$$ \Delta d = \Delta N \cdot \frac{2 \pi r}{N_{rev}} $$

The center displacement $\Delta s$ and heading change $\Delta \theta$ are:
$$ \Delta s = \frac{\Delta d_L + \Delta d_R}{2}, \quad \Delta \theta \approx \omega_{gyro} \cdot \Delta t $$

The global position is updated iteratively:
$$ x_{k+1} = x_k + \Delta s \cos(\theta_{k+1}), \quad y_{k+1} = y_k + \Delta s \sin(\theta_{k+1}) $$

## Inputs and Expected Output
- **Inputs**: Encoder values (`L`, `R`), wheel radius, ticks per revolution, gyroscope reading (`gyro()`), time step (`dt`).
- **Output**: Submits an estimated `Pose2d(x, y, yaw)` to the robot.

## Score obtained with this approach

97.823 / 100