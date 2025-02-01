package com.team973.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveComposable {
  public static DriveComposable compose(
      DriveComposable translationProvider, DriveComposable rotationProvider) {
    return new DriveComposable() {
      public void init() {
        translationProvider.init();
        rotationProvider.init();
      }

      @Override
      public ChassisSpeeds getOutput() {
        ChassisSpeeds output = new ChassisSpeeds();

        output.vxMetersPerSecond = translationProvider.getOutput().vxMetersPerSecond;
        output.vyMetersPerSecond = translationProvider.getOutput().vyMetersPerSecond;
        output.omegaRadiansPerSecond = rotationProvider.getOutput().omegaRadiansPerSecond;

        return output;
      }
    };
  }

  public abstract void init();

  public abstract ChassisSpeeds getOutput();
}
