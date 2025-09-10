package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.auto.commands.PickupAlgaeCommand;
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

public class BabyBird extends AutoMode {
  public BabyBird(Logger logger, DriveController drive, Superstructure superstructure) {
    super(
        logger,
        new Pose2d(8.487, 1.363, Rotation2d.fromDegrees(180)),
        new DriveTrajectoryCommand(drive, "Babybird", logger.subLogger("Babybird")),
        new ScoreCoralCommand(drive, superstructure, ReefFace.D, ReefLevel.L_4, ReefSide.Left),
        new PickupAlgaeCommand(drive, superstructure, ReefFace.D));
  }

  public String getName() {
    return "Babybird";
  }
}
