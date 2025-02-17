package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TaxiAuto extends AutoMode {
  public TaxiAuto(Logger logger, DriveController drive, Claw claw) {
    super(
        logger,
        new Pose2d(7.6, 4, Rotation2d.fromDegrees(180)),
        new DriveTrajectoryCommand(drive, "C-4"));
  }

  public String getName() {
    return "Taxi Auto";
  }
}
