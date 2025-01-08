package com.team973.lib.util;

public class SwerveModuleConfig {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final double angleOffset;

  public SwerveModuleConfig(
      int driveMotorID, int angleMotorID, int cancoderID, double angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = cancoderID;
    this.angleOffset = angleOffset;
  }
}
