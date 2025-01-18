package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TestAuto extends AutoMode {
  public TestAuto(Logger logger, DriveController drive) {
    super(
        logger,
        new Pose2d(7.6, 4, Rotation2d.fromDegrees(180)),
        new DriveTrajectoryCommand(logger.subLogger("PF-HP-L2"), drive, "PF-HP-L2"),
        new DriveTrajectoryCommand(logger.subLogger("PF-L2-HP"), drive, "PF-L2-HP"),
        new DriveTrajectoryCommand(logger.subLogger("PF-HP-R2"), drive, "PF-HP-R2"),
        new DriveTrajectoryCommand(logger.subLogger("PF-R2-HP"), drive, "PF-R2-HP"));
  }

  public String getName() {
    return "Test Auto";
  }
}
