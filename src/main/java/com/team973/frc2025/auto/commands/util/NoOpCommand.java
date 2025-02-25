package com.team973.frc2025.auto.commands.util;

import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class NoOpCommand extends AutoCommand {
  public void init() {}

  public void run(Alliance alliance) {}

  public void log(Alliance alliance) {}

  public boolean isCompleted() {
    return true;
  }

  public void postComplete(boolean interrupted) {}
}
