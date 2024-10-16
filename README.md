# Mobile_robotics
Design algorithm for Mobile Robots

$$v = r \omega$$

$$rpm = \frac{v \cdot 60}{2 \pi r}$$

where:
- `v` is the velocity of the wheel
- `r` is the radius of the wheel
- `ω` is the angular velocity (motorVelocity) of the wheel
- `rpm` is the revolutions per minute of the wheel
- `π` is the revolutions per minute of the object

## Calculation of Angular Displacement from RPM and Time Interval

### Step 1: Convert RPM to Angular Velocity
To calculate the motor’s angular displacement, we first need to convert the rotational speed from RPM to angular velocity in radians per second.

#### The formula for converting RPM to angular velocity is:

$$omega= RPM×\frac{2 \pi}{60}$$
or,
$$omega = RPM×0.1047$$

### After calculating the angular velocity, we divide by the gear ratio to adjust for the gearing system. The final formula for motor velocity is:

$$motorVelocity=\frac{RPM×0.1047}{gearRatio}$$

##### For the left and right motors, respectively:
$$LeftmotorVelocity=\frac{LeftRPM×0.1047}{gearRatio}$$
$$RightmotorVelocity=\frac{RightRPM×0.1047}{gearRatio}$$

### gear ratio might be (30:1, 20:1, 60:1, ....)


### Step 2: Calculate Angular Displacement
Once we have the angular velocity, the next step is to calculate the angular displacement. Angular displacement refers to how far the motor has rotated during a given time interval.

#### The formula for angular displacement is:
$$θ = ω×Δt$$

#### Thus, the formulas for the left and right motor angular displacements are:
$$leftAngularDisplacement = leftMotorVelocity×Δt$$
$$rightAngularDisplacement = rightMotorVelocity×Δt$$

where:
- `θ` is the angular displacement
- `Δt` represents the time interval (delta_seconds).
  
