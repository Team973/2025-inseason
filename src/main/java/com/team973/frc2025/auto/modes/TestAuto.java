package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.ScoreCoralCommand;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TestAuto extends AutoMode {
  public TestAuto(Logger logger, DriveController drive, Superstructure superstructure) {
    super(
        logger,
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new ScoreCoralCommand(drive, superstructure, ReefFace.A, ReefLevel.L_1));
  }

  public String getName() {
    return "Test Auto";
  }
}
