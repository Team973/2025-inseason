package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.auto.commands.ScoreCoralCommand;
import com.team973.frc2025.auto.commands.util.BlockingLambdaCommand;
import com.team973.frc2025.auto.commands.util.BranchCommand;
import com.team973.frc2025.auto.commands.util.LambdaCommand;
import com.team973.frc2025.auto.commands.util.NoOpCommand;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.GamePiece;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefSide;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LeftSideAuto extends AutoMode {
  private boolean m_babybird;

  public LeftSideAuto(
      Logger logger, Superstructure superstructure, DriveController drive, boolean doBabyBird) {
    super(
        logger,
        new Pose2d(7.3, 5.7, Rotation2d.fromDegrees(180)),
        new BranchCommand(
            logger, doBabyBird, new DriveTrajectoryCommand(drive, "Babybird-L"), new NoOpCommand()),
        new ScoreCoralCommand(drive, superstructure, ReefFace.E, ReefLevel.L_4, ReefSide.Right),
        new DriveTrajectoryCommand(drive, "E-HP"),
        new BlockingLambdaCommand(() -> superstructure.getSeesCoral(), 0.35),
        new DriveTrajectoryCommand(drive, "HP-F"),
        new ScoreCoralCommand(drive, superstructure, ReefFace.F, ReefLevel.L_4, ReefSide.Right),
        new DriveTrajectoryCommand(drive, "F-HP"),
        new ScoreCoralCommand(drive, superstructure, ReefFace.F, ReefLevel.L_4, ReefSide.Left),
        new LambdaCommand(
            () -> {
              superstructure.setGamePieceMode(GamePiece.Algae);
              superstructure.setTargetReefLevel(ReefLevel.AlgaeLow);
              drive.getDriveWithLimelight().setTargetSide(ReefSide.Center);
            }));
    m_babybird = doBabyBird;
  }

  public String getName() {
    if (m_babybird) {
      return "Left Side Babybird Auto";
    } else {
      return "Left Side Auto";
    }
  }
}
