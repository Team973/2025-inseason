package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.auto.commands.ScoreCoralCommand;
import com.team973.frc2025.auto.commands.util.BlockingLambdaCommand;
import com.team973.frc2025.auto.commands.util.BranchCommand;
import com.team973.frc2025.auto.commands.util.NoOpCommand;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefSide;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ThreePieceAuto extends AutoMode {
  public ThreePieceAuto(
      Logger logger, Superstructure superstructure, DriveController drive, boolean doBabyBird) {
    super(
        logger,
        new Pose2d(7.18, 5.6, Rotation2d.fromDegrees(180)),
        new BranchCommand(
            logger,
            doBabyBird,
            new DriveTrajectoryCommand(drive, "Babybird-L", logger.subLogger("Babybird-L")),
            new NoOpCommand()),
        new ScoreCoralCommand(drive, superstructure, ReefFace.F, ReefLevel.L_4, ReefSide.Right),
        new DriveTrajectoryCommand(drive, "F-HPL", logger.subLogger("F-HPL")),
        new BlockingLambdaCommand(() -> superstructure.getSeesCoral(), 0.35),
        new DriveTrajectoryCommand(drive, "HPL-A", logger.subLogger("HPL-A")),
        new ScoreCoralCommand(drive, superstructure, ReefFace.A, ReefLevel.L_4, ReefSide.Right),
        new DriveTrajectoryCommand(drive, "A-HPL", logger.subLogger("A-HPL")),
        new BlockingLambdaCommand(() -> superstructure.getSeesCoral(), 0.35),
        new DriveTrajectoryCommand(drive, "HPL-A", logger.subLogger("HPL-A 2")),
        new ScoreCoralCommand(drive, superstructure, ReefFace.A, ReefLevel.L_4, ReefSide.Left));
  }

  public String getName() {
    return "Three Piece Auto";
  }
}
