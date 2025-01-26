package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.Claw;
import com.team973.lib.util.AutoCommand;

public class WaitUntillCoralSeeStateCommand extends AutoCommand {
  private final Claw m_claw;
  private final boolean m_waitUntilSensorSeesCoral;

  public WaitUntillCoralSeeStateCommand(Claw claw, boolean waitUntilSensorSeesCoral) {
    m_claw = claw;
    m_waitUntilSensorSeesCoral = waitUntilSensorSeesCoral;
  }

  public boolean coralCorrectPlace() {
    return (m_claw.frontBannerSensorSeesCoral() && !m_claw.backBannerSensorSeesCoral());
  }

  @Override
  public void init() {}

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return (m_waitUntilSensorSeesCoral && coralCorrectPlace())
        || (!m_waitUntilSensorSeesCoral && !coralCorrectPlace());
  }

  @Override
  public void postComplete(boolean interrupted) {}

  public void log() {}
}
