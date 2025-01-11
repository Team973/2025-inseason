package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.Claw;
import com.team973.lib.util.AutoCommand;

public class ClawCommand extends AutoCommand {
  private final Claw m_claw;
  private final Claw.ControlStatus m_state;

  public ClawCommand(Claw claw, Claw.ControlStatus state) {
    m_claw = claw;
    m_state = state;
  }

  @Override
  public void init() {
    m_claw.setControl(m_state);
  }

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return true;
  }

  @Override
  public void postComplete(boolean interrupted) {}

  public void log() {}
}
