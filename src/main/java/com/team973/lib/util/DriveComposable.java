package com.team973.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveComposable {
  private boolean m_firstRun = true;

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
