package com.team973.frc2025.auto.modes;

import com.team973.frc2025.auto.commands.DriveTrajectoryCommand;
import com.team973.frc2025.auto.commands.util.SequentialCommand;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.Logger;

public class TaxiAuto extends SequentialCommand {
  public TaxiAuto(Logger logger, DriveController drive) {
    super(logger, new DriveTrajectoryCommand(drive, "C-4"));
  }
}
