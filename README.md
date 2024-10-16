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

$$omega= RPM×\frac{2 \pi}{60}$$
or,
$$omega = RPM×0.1047$$

### After calculating the angular velocity, we divide by the gear ratio to adjust for the gearing system. The final formula for motor velocity is:

$motorVelocity=\frac{RPM×0.1047}{gear_ratio}$$

### gear ratio might be (30:1, 20:1, 60:1, ....)
