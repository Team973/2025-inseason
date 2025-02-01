package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveTestAuto extends AutoMode {
  public DriveTestAuto(Logger logger, DriveController drive) {
    super(
        logger,
        new Pose2d(5, 5, Rotation2d.fromDegrees(180)),
        new DriveTrajectoryCommand(drive, "Test"));
  }

  public String getName() {
    return "Drive Test Auto";
  }
}
