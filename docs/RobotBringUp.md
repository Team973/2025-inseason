# Robot Bring Up

## 1. Swerve Module Constants

- ID swerve motors
- Find the gear ratios (drive and turn)
- Find the wheel diameter
- Find the track width
- Find the wheel base
- Find the motor inversion

## 2. [Find Swerve Module Offsets](UpdatingModuleOffsets.md)

## 3. Tune Swerve Modules

Drive the robot in teleop and compare the target drive velocity (log key is `mod0/Target Drive Velocity RPS`) to the actual velocity (log key is `mod0/Drive Motor/Velocity`). Also compare the target turn angle (log key is `mod0/Target Angle Rot`) and actual turn angle (log key is `mod0/Angle Motor/Position Rot`).

For tuning the drive motor, start by zeroing all PID gains except for feed forward. If the actual doesn't meet the target, multiply it by the target divided by the actual. Repeat this until the actual is barely below the target.

Next, give the kP a small value so that the actual gets a little bit closer to the target. Then, repeat doubling the kP and testing the drive until the actual starts oscillating around the target. Then, use the average of the oscillating kP and the previous stable kP. Repeat this until you get the fastest stable kP possible.

For tuning the turn motor, start with kP (there is no feed forward for turn). If the actual undershoots, increase kP. If it overshoots, average between the current kP and the previous stable kP. Repeat this until you have the fastest stable kP possible.

## 4. Tune the Holonomic Controller

Create a test auto that involves translation, rotation, and changes in direction. Compare the target positions to the actual positions.
