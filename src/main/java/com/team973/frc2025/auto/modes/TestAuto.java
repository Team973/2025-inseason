package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TestAuto extends AutoMode {
  public TestAuto(Logger logger, DriveController drive, Claw claw) {
    super(
        logger,
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        // new ClawCommand(claw, Claw.ControlStatus.IntakeAndHold),
        // new WaitUntillCoralSeeStateCommand(claw, true),
        // new DriveTrajectoryCommand(drive, "PF-HP-L2"),
        // new ClawCommand(claw, Claw.ControlStatus.Score),
        // new WaitUntilCoralScoredComand(claw),
        // new DriveTrajectoryCommand(drive, "PF-L2-HP"),
        // new ClawCommand(claw, Claw.ControlStatus.IntakeAndHold),
        // new WaitUntillCoralSeeStateCommand(claw, true),
        // new DriveTrajectoryCommand(drive, "PF-HP-R2"),
        // new ClawCommand(claw, Claw.ControlStatus.Score),
        // new WaitUntilCoralScoredComand(claw),
        new DriveTrajectoryCommand(drive, "Drive-Test"));
  }

  public String getName() {
    return "Test Auto";
  }
}
