package com.team973.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveComposable {
  private boolean m_firstRun = true;

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

  /**
   * Initializes the composable and sets first run to true. Called by the drive controller when the
   * composable is selected.
   */
  public void start() {
    m_firstRun = true;
    init();
  }

  /** Initializes the composable. Overridden by the composable and called by the start method. */
  protected abstract void init();

  public void firstRunComplete() {
    m_firstRun = false;
  }

  public boolean getFirstRun() {
    return m_firstRun;
  }

  public abstract ChassisSpeeds getOutput();
}
