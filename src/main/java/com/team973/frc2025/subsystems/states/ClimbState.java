package com.team973.frc2025.subsystems.states;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.util.SubsystemState;

public class ClimbState implements SubsystemState {
  private final Superstructure m_superstructure;

  public ClimbState(Superstructure superstructure) {
    m_superstructure = superstructure;
  }

  public void init() {}

  public void run() {
    m_superstructure.clawOff();

    m_superstructure.setTargetReefLevel(ReefLevel.Horizontal);

    m_superstructure.armTargetReefLevel();
    m_superstructure.elevatorTargetReefLevel();
    m_superstructure.wristTargetReefLevel();

    m_superstructure.setManualArmivator(true);
  }
}
