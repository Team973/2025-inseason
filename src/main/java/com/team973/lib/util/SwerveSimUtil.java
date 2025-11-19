package com.team973.lib.util;

public class SwerveSimUtil {
  public static SwerveModuleConfig sanitizeModuleConfig(SwerveModuleConfig config) {
    return new SwerveModuleConfig(config.driveMotorID, config.angleMotorID, config.cancoderID, 0.0);
  }
}
