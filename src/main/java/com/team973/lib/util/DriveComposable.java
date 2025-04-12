package com.team973.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveComposable {
  public static DriveComposable compose(
      DriveComposable translationProvider, DriveComposable rotationProvider) {
    return new DriveComposable() {
      public void init(
          ChassisSpeeds previousChassisSpeeds, boolean robotIsAutonomous, Pose2d currentPose) {
        translationProvider.init(previousChassisSpeeds, robotIsAutonomous, currentPose);
        rotationProvider.init(previousChassisSpeeds, robotIsAutonomous, currentPose);
      }

      public void exit() {
        translationProvider.exit();
        rotationProvider.exit();
      }

      @Override
      public ChassisSpeeds getOutput(Pose2d currentPose, Rotation2d angularVelocity) {
        ChassisSpeeds output = new ChassisSpeeds();

        output.vxMetersPerSecond =
            translationProvider.getOutput(currentPose, angularVelocity).vxMetersPerSecond;
        output.vyMetersPerSecond =
            translationProvider.getOutput(currentPose, angularVelocity).vyMetersPerSecond;
        output.omegaRadiansPerSecond =
            rotationProvider.getOutput(currentPose, angularVelocity).omegaRadiansPerSecond;

        return output;
      }
    };
  }

  public abstract void init(
      ChassisSpeeds previousChassisSpeeds, boolean robotIsAutonomous, Pose2d currentPose);

  public abstract void exit();

  public abstract ChassisSpeeds getOutput(Pose2d currentPose, Rotation2d angularVelocity);
}
