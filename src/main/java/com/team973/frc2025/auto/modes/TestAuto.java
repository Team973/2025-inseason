package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.ClawCommand;
import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.CommandOnEvent;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TestAuto extends AutoMode {
  public TestAuto(Logger logger, DriveController drive) {
    super(
        logger,
        new Pose2d(7.6, 4, Rotation2d.fromDegrees(180)),
        new DriveTrajectoryCommand(
            drive,
            "PF-HP-L2",
            new CommandOnEvent("Shoot", new ClawCommand(null, Claw.ControlStatus.Shoot))),
        new DriveTrajectoryCommand(drive, "PF-L2-HP"),
        new DriveTrajectoryCommand(drive, "PF-HP-R2"),
        new DriveTrajectoryCommand(drive, "PF-R2-HP"));
  }

  public String getName() {
    return "Test Auto";
  }
}
