package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.auto.commands.PickupAlgaeCommand;
import com.team973.frc2025.auto.commands.ScoreAlgaeInNetCommand;
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

public class CenterAuto extends AutoMode {
  public CenterAuto(Logger logger, Superstructure superstructure, DriveController drive) {
    super(
        logger,
        new Pose2d(7.18, 4, Rotation2d.fromDegrees(180)),
        new ScoreCoralCommand(drive, superstructure, ReefFace.D, ReefLevel.L_4, ReefSide.Right),
        new DriveTrajectoryCommand(drive, "D-Backoff", logger.subLogger("D-Backoff")),
        new PickupAlgaeCommand(drive, superstructure, ReefFace.D, ReefLevel.AlgaeLow),
        new ScoreAlgaeInNetCommand(drive, superstructure));
  }

  public String getName() {
    return "Center Auto";
  }
}
