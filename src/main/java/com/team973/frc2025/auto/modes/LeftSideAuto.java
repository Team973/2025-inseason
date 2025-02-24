package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.ScoreCoralCommand;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefSide;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LeftSideAuto extends AutoMode {
  public LeftSideAuto(Logger logger, Superstructure superstructure, DriveController drive) {
    super(
        logger,
        new Pose2d(8, 5.7, Rotation2d.fromDegrees(180)),
        new ScoreCoralCommand(drive, superstructure, ReefFace.E, ReefLevel.L_4, ReefSide.Left)
        // new DriveTrajectoryCommand(drive, "EL-HP"),
        // new BlockingLambdaCommand(() -> superstructure.getSeesCoral(), 2),
        // new ScoreCoralCommand(drive, superstructure, ReefFace.E, ReefLevel.L_4, ReefSide.Right)
        );
  }

  public String getName() {
    return "Left Side Auto";
  }
}
