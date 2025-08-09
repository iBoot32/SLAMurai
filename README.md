![SLAM Robot](slam_robot.png)

# SLAMurai Autonomous Robot

This project is an open-source implementation of a SLAM (Simultaneous Localization and Mapping) robot.
All CAD and software is contained within this repo.

## Features

- Real-time mapping
- Autonomous navigation

&nbsp;

# Detailed Robot Design
### Payload:
- 4x motors: 0.25kg each = 1.0kg total
- 4x 100mm omni wheels: 0.3kg each = 1.2kg total
- 1x 6000mAhlipo battery: 0.55kg
- 1x NVIDIA TX2: 0.5kg
- 1x Intel RealSense D455: 0.3kg
- 3x chassis plates: 0.3kg total
- 1x LIDAR: 0.2kg

**Total weight: 4.5kg rounded**

### Desired robot specs:
- Speed: 0.75 m/s
- Acceleration: 0.3 m/s²
- Max payload: 4.5 kg (just the total robot weight)
- Wheels: 100mm omni wheel in 90° configuration (two motors drive the robot forward, two motors drive the robot sideways)
- Worst case surface: Concrete (Rolling resistance ~0.03)
- Torque safety factor: 2

## Selecting Motor Parameters:
#### Required torque:

```math
\text{F}_{\text{roll}} = \mu \cdot \text{F}_{\text{normal}} = 0.03 \cdot 4.5 \text{kg} \cdot 9.81 \text{m/s}^2 = 1.32 \text{N}
```

```math
\text{F}_{\text{accel}} = 4.5 \text{kg} \cdot 0.3 \text{m/s}^2 = 1.35 \text{N}
```

```math
\text{F}_{\text{total}} = 1.32 \text{N} + 1.35 \text{N} = \boxed{2.67 \text{N}}
```

```math
\tau = \text{F}_{\text{total}} \cdot \text{r}_{\text{wheel}} = 2.67 \text{N} \cdot 0.05 \text{m} = \boxed{0.134 \text{Nm}}
```

This gives us the total torque required for the robot to meet our acceleration target. However, we need the **worst-case torque per motor**. One might assume it splits evenly between two motors when moving forward/backward or left/right. But could diagonal movement need more?

<img src="image.png" alt="alt text" width="300" height="220" />

If each motor provides torque $T$, the top/bottom motors contribute $2 T \cos(\theta)$ and left/right motors $2 T \cos(90^\circ - \theta)$, where $\theta$ is the angle of travel. We can solve for $T$ to achieve some total torque $\tau$:

```math
\tau = 2 T \cos(\theta) + 2 T \cos(90^\circ - \theta)
```

```math
T = \frac{\tau}{2 (\cos(\theta) + \sin(\theta))}
```

$T$ peaks when $\cos(\theta) + \sin(\theta)$ is minimized. We only consider $0 \leq \theta \leq 90^\circ$ since if the angle is $> 90^\circ$, we need to switch the direction of the top/bottom motors, yielding an angle $0 \leq \theta \leq 90^\circ$ again. Therefore the maximum of $T$ occurs when $\theta = 0^\circ$ or $90^\circ$. So yes, the worst case is moving purely along the top/bottom or left/right axes, splitting torque evenly:

```math
T = \frac{\tau}{2} = \frac{0.134 \text{Nm}}{2} = \boxed{0.067 \text{Nm}} \cdot 2_\text{(safety factor)} \approx \boxed{0.134 \text{Nm worst case per motor}}
```

```math
0.134 \text{Nm} \cdot \frac{10.197 \text{kg*cm}}{1 \text{Nm}} \approx \boxed{1.37 \text{kg*cm}}
```

#### Then for required RPM:

```math
\text{RPM} = \frac{\text{Target velocity} \cdot 60}{2 \pi r} = \frac{0.75 \text{m/s} \cdot 60}{2 \pi \cdot 0.05 \text{m}} \approx \boxed{143.24 \text{RPM}}
```

```math
\boxed{\text{So we need motors with at least 0.134 Nm torque and 143 RPM}}
```

## Motor Selection
I searched for a while and I either found motors with too low torque, or shipped from China and costed a fortune. Perhaps worse, motors on Amazon are reasonably priced and free shipping, but the vendors despise listing torque (much less spec sheets) for some reason.

Eventually I found [this motor](https://www.amazon.com/CQRobot-Ocean-6V-3W-20RPM-40-oz-12V-6W-40RPM-70/dp/B08ZK6QCCL/ref=pd_ci_mcx_di_int_sccai_cn_d_sccl_2_1/138-6137927-8750040?pd_rd_w=Zg5pj&content-id=amzn1.sym.751acc83-5c05-42d0-a15e-303622651e1e&pf_rd_p=751acc83-5c05-42d0-a15e-303622651e1e&pf_rd_r=95C2GMX20CP7G5X6CPWA&pd_rd_wg=w4uHO&pd_rd_r=e108f911-329a-4624-8c83-7a904740f90a&pd_rd_i=B08X3CDZRF&th=1) which boasts many configurations, and their 177RPM seemed pretty good for $34. They only list stall torque as 23kg*cm and no-load speed as 177RPM, but that's actually enough to calculate the torque at any RPM, since torque decreases linearly with RPM:

```math
\tau_{\text{at RPM}} = \frac{\tau_{\text{stall}} - 0}{0 - \text{No-load RPM}} \cdot \text{RPM} + \tau_{\text{stall}}
```
```math
\tau_{\text{at RPM}} = \tau_{\text{stall}} \left(1 - \frac{\text{RPM}}{\text{No-load RPM}}\right)
```

```math
\tau_{\text{at 143.24 RPM}} = 23 \text{kg*cm} \left(1 - \frac{143.24}{177}\right) \approx 4.3 \text{kg*cm}
```

This is over 3 times our required torque of 1.37 kg*cm, so we are good there. But let's check the current draw. A rule of thumb is to not run motors at more than 25% of their stall current (5.5A for this motor). Because torque and current are also linear, we can calculate the current at 143.24 RPM (assuming 0.2A no-load current from data sheet):

```math
I_{\text{at 143.24 RPM}} = I_{\text{no-load}} + (I_{\text{stall}} - I_{\text{no-load}}) \left(1 - \frac{\text{RPM}}{\text{No-load RPM}}\right) = 0.2 \text{A} + (5.5 \text{A} - 0.2 \text{A}) \left(1 - \frac{143.24}{177}\right) \approx 1.2 \text{A}
```

```math
100 * \frac{1.2 \text{A}}{5.5 \text{A}} \approx 21.8\%
```

Awesome, this means the motor can meet our desired RPM at the required torque, and we are well below the 25% rule of thumb for current draw. We could even increase the robot's weight to 14kg and still be under the provided torque:

```math
T_{\text{motor}} = \frac{\left( \left( 0.03 \cdot 14 \cdot 9.81 \right) + \left( 14 \cdot 0.3 \right) \right) \cdot 0.05}{2} \cdot 2_\text{(safety factor)} \cdot 10.197 \approx 4.24 \text{kg*cm worst case per motor}
```

### A quick side tangent to verify our selection:

If we wished to possibly squeeze more torque out of the motors, in this specific case we can actually pick a lower gear ratio (higher RPM), as shown by this plot:

<img src="torque_rpm.png" alt="alt text" width="400" height="300" />

However if we pick the next higher RPM of 228, our current at 143.24 RPM would be:

```math
I_{\text{at 143.24 RPM}} = 0.2 \text{A} + (5.5 \text{A} - 0.2 \text{A}) \left(1 - \frac{143.24}{228}\right) \approx 2.13 \text{A} \rightarrow \frac{2.13 \text{A}}{5.5 \text{A}} \approx 38.7\%
```

This is above the 25% rule of thumb, so we will stick with the 177 RPM gearing.

### Final Thing to Check

We are building an autonomous robot, so we wish to use encoders to obtain odometry. The motor we selected DOES have an encoder which outputs 64 counts per revolution before the gearbox. At our gear ratio of 56.3:1, this gives us $64 \cdot 56.3 \approx 3600$ counts per revolution of the wheel. To find the linear "accuracy" given our 100mm diameter wheels, we convert to circumference and find the distance per count:

```math
\text{Circumference} = 2 \pi r = 2 \pi \cdot 0.05 \approx 0.314 \text{m}
```

```math
\text{Distance per count} = \frac{\text{Circumference}}{\text{Counts per revolution}} \approx \frac{0.314 \text{m}}{3600} \approx 0.087 \text{mm/count}
```

This means the robot can track its movement with sub-millimeter accuracy. So slipping will be the limiting factor, not the motor resolution. In terms of RPM, torque, current, and encoder resolution, we are good to go with this motor.

## Driver Selection
This is way easier than motor selection. I prefer a dual-channel driver since I'd rather buy two drivers than four. People recommend the Cytron MDD10A, which provides 10A continuous current per motor- nearly double our 5.5A stall current. Unfortunately it does not sense current, so we can only limit RPM and not torque without additional circuitry. But it is cheap and works well for our purposes.

## Battery Selection
Desired specs:
- Voltage: 12V (compatible with motors and TX2, so avoids needing a buck converter)
- Runtime: 1 hour (at 0.75 m/s and 0.3 m/s² acceleration with 4.5kg payload)

Let's find when the robot will use the most torque:

```math
\max_{\theta \in [0^\circ, 90^\circ]} \left( \tau \left(\theta\right) \right) = \max_{\theta \in [0^\circ, 90^\circ]} \left(2 T \cos(\theta) + 2 T \cos(90^\circ - \theta)\right) = 2 T \sqrt{2}
```

This happens when $\theta = 45^\circ$. So the torque used per motor at this angle is:

```math
T = \frac{\tau}{2 \sqrt{2}} = \frac{1.37 \text{kg*cm}}{2 \sqrt{2}} \approx 0.484 \text{kg*cm}
```
And the current draw at this torque is:

```math
I = I_{\text{no-load}} + (\frac{I_\text{stall} - I_\text{no-load}}{T_\text{stall}}) T
```

```math
I = 0.2 \text{A} + \left(\frac{5.5 \text{A} - 0.2 \text{A}}{23 \text{kg*cm}}\right) \cdot 0.484 \text{kg*cm} \approx 0.2 + 0.1 \approx 0.3 \text{A}
```

```math
\text{Total current draw} = 4 \cdot 0.3 \text{A} = 1.2 \text{A}
```


The TX2 draws about 15W at 12V, or 1.25A. The RealSense D455 runs at 5V and probably 3.5W max, which is 0.7A. Finally, the RPLIDAR A1M8 draws around 100mA. So our overly-conservative total continuous current draw is:

```math
I_{\text{total}} = 1.2 \text{A} + 1.25 \text{A} + 0.7 \text{A} + 0.1 \text{A} = 3.25 \text{A}
```

To run for 1 hour, we need a battery with at least:

```math
\text{Capacity} = I_{\text{total}} \cdot \text{Time} = 3.25 \text{A} \cdot 1 \text{h} = 3.25 \text{Ah} = 3250 \text{mAh}
```

Our 6000mAh battery is more than sufficient, and leaves a good safety margin.


## License

MIT License
