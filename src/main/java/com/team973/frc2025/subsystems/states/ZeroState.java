package com.team973.frc2025.subsystems.states;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.lib.util.SubsystemState;

public class ZeroState implements SubsystemState {
  private final Superstructure m_superstructure;

  public ZeroState(Superstructure superstructure) {
    m_superstructure = superstructure;
  }

  public void init() {}

  public void run() {
    m_superstructure.zeroElevator();

    m_superstructure.wristStow();
    m_superstructure.armStow();
    m_superstructure.clawIntake();
  }
}
