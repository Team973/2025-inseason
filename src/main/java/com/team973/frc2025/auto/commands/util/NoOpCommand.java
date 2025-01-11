package com.team973.frc2025.auto.commands.util;

import com.team973.lib.util.AutoCommand;

public class NoOpCommand extends AutoCommand {
  public void init() {}

  public void run() {}

  public void log() {}

  public boolean isCompleted() {
    return true;
  }

  public void postComplete(boolean interrupted) {}
}
