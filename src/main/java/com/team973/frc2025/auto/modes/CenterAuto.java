package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.PickupAlgaeCommand;
import com.team973.frc2025.auto.commands.ScoreAlgaeInNetCommand;
import com.team973.frc2025.auto.commands.ScoreCoralCommand;
import com.team973.frc2025.auto.commands.util.LambdaCommand;
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
        new PickupAlgaeCommand(drive, superstructure, ReefFace.D),
        new LambdaCommand(() -> superstructure.setTargetReefLevel(ReefLevel.AlgaeHigh)),
        new ScoreAlgaeInNetCommand(drive, superstructure));
    // Scoring the second algae caused the robot to finish auto on the taxi line, so in order to
    // guarantee us taxi points we are no longer retrieving or scoring it.
    // new PickupAlgaeCommand(drive, superstructure, ReefFace.E),
    // new ScoreAlgaeInNetCommand(drive, superstructure));
  }

  public String getName() {
    return "Center Auto";
  }
}
