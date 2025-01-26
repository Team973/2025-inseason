package com.team973.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveComposable {
  private boolean m_firstRun = true;

  public static DriveComposable compose(
      DriveComposable translationProvider, DriveComposable rotationProvider) {
    return new DriveComposable() {
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

  public void init() {
    m_firstRun = true;
  }

  public void firstRunComplete() {
    m_firstRun = false;
  }

  public boolean getFirstRun() {
    return m_firstRun;
  }

  public abstract ChassisSpeeds getOutput();
}
