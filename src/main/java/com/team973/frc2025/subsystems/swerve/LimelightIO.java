package com.team973.frc2025.subsystems.swerve;

import com.team973.frc2025.subsystems.swerve.MegaTagSupplier.MegaTagReceiver;

public interface LimelightIO {
  public void addReceiver(MegaTagReceiver newReceiver);

  public void syncSensors();

  public void log();

  public void doCycle();
}
