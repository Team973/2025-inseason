package com.team973.frc2025.auto.modes;

import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;

public class NoAuto extends AutoMode {
  public NoAuto(Logger logger) {
    super(logger, new Pose2d());
  }

  public String getName() {
    return "No Auto";
  }
}
