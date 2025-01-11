# Debugging Drive
## Reset Odometry Button
If the reset odometry button causes the robot to rotate, open AdvantageScope and make sure that the robot yaw and target angle are both being set to zero when the button is pressed. If they are, make sure that the pigeon is working correctly and giving values that make sense. If they aren't, look through the `resetOdometry` functions in Drive and make sure that there aren't any logic errors with setting the yaw offset. (The yaw offset should be set to the current raw yaw.)
## Modules are driving in different directions
If the modules are going in different directions, the angle offsets are probably wrong. Check [Updating Module Offsets](UpdatingModuleOffsets.md)
## Robot is driving too slow/turning poorly
Open PhoenixTuner and make sure that all of the drive motors have the correct IDs. Blink them to make sure they correspond to the right motors.
## Robot drives incorrect distance in auto
Find the `WHEEL_DIAMETER_METERS` constant in `DriveInfo`. Multiply it by the distance driven divided by the desired distance.
