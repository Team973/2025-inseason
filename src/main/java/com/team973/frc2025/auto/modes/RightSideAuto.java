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

public class RightSideAuto extends AutoMode {
  public RightSideAuto(
      Logger logger, Superstructure superstructure, DriveController drive, boolean doBabyBird) {
    super(
        logger,
        new Pose2d(7.3, 1.3, Rotation2d.fromDegrees(180)),
        new BranchCommand(
            logger,
            doBabyBird,
            new DriveTrajectoryCommand(drive, "Babybird-Left"),
            new NoOpCommand()),
        new ScoreCoralCommand(drive, superstructure, ReefFace.C, ReefLevel.L_4, ReefSide.Right),
        new DriveTrajectoryCommand(drive, "C-HP"),
        new BlockingLambdaCommand(() -> superstructure.getSeesCoral(), 0),
        new DriveTrajectoryCommand(drive, "HP-B"),
        new ScoreCoralCommand(drive, superstructure, ReefFace.B, ReefLevel.L_4, ReefSide.Left),
        new DriveTrajectoryCommand(drive, "B-HP"),
        new DriveTrajectoryCommand(drive, "HP-B"),
        new ScoreCoralCommand(drive, superstructure, ReefFace.B, ReefLevel.L_4, ReefSide.Right));
  }

  public String getName() {
    return "right Side Auto";
  }
}
