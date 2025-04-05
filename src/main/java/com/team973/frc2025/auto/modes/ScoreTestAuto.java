package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
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

public class ScoreTestAuto extends AutoMode {
  public ScoreTestAuto(Logger logger, DriveController drive, Superstructure superstructure) {
    super(
        logger,
        new Pose2d(2, 4, Rotation2d.fromDegrees(0)),
        new ScoreCoralCommand(drive, superstructure, ReefFace.A, ReefLevel.L_4, ReefSide.Left),
        new DriveTrajectoryCommand(drive, "Score-Test", logger.subLogger("Score-Test")));
  }

  public String getName() {
    return "Score Test Auto";
  }
}
