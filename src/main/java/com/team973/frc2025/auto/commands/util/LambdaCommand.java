package com.team973.frc2025.auto.commands.util;

import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LambdaCommand extends AutoCommand {
  private final Runnable m_lambda;
  private boolean m_ran = false;

  public LambdaCommand(Runnable lambda) {
    m_lambda = lambda;
  }

  public void init() {}

  public void run(Alliance alliance) {
    if (!m_ran) {
      m_lambda.run();
      m_ran = true;
    }
  }

  public void log(Alliance alliance) {}

  public boolean isCompleted() {
    return true;
  }

  public void postComplete(boolean interrupted) {}
}
