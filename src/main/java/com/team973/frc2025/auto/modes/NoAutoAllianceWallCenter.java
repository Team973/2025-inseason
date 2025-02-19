package com.team973.frc2025.auto.modes;

import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class NoAutoAllianceWallCenter extends AutoMode {
  public NoAutoAllianceWallCenter(Logger logger) {
    // HACK: These X,Y are for red wall but the 0 is for blue alliance.
    // Blue gets corrected, byt X and Y still need to be corrected for alliance.
    super(logger, new Pose2d(17.13, 3.95, Rotation2d.fromDegrees(0)));
  }

  public String getName() {
    return "NoAutoAllianceWallCenter";
  }
}
