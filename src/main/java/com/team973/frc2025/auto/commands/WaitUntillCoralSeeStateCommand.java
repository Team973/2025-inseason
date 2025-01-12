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

  @Override
  public void init() {}

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return (m_waitUntilSensorSeesCoral && m_claw.sensorSeeCoral())
        || (!m_waitUntilSensorSeesCoral && !m_claw.sensorSeeCoral());
  }

  @Override
  public void postComplete(boolean interrupted) {}

  public void log() {}
}
