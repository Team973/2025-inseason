package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.Claw;
import com.team973.lib.util.AutoCommand;

public class WaitUntilCoralScoredComand extends AutoCommand {
  private final Claw m_claw;

  public WaitUntilCoralScoredComand(Claw claw) {
    m_claw = claw;
  }

  @Override
  public void init() {}

  @Override
  public void run() {}

  @Override
  public boolean isCompleted() {
    return m_claw.motorAtTarget();
  }

  @Override
  public void postComplete(boolean interrupted) {}

  public void log() {}
}
