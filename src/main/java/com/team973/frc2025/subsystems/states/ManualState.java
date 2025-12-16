package com.team973.frc2025.subsystems.states;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.lib.util.SubsystemState;

public class ManualState implements SubsystemState {
  private final Superstructure m_superstructure;

  public ManualState(Superstructure superstructure) {
    m_superstructure = superstructure;
  }

  public void init() {
    m_superstructure.setManualScore(false);
    m_superstructure.setManualIntake(true);
  }

  public void run() {
    if (m_superstructure.getManualScore()) {
      m_superstructure.clawScore();
    } else if (m_superstructure.getManualIntake()) {
      m_superstructure.clawIntake();
    } else {
      m_superstructure.clawReverse();
    }

    if (m_superstructure.getManualArmivator()) {
      m_superstructure.armTargetReefLevel();
      m_superstructure.elevatorTargetReefLevel();
      m_superstructure.wristTargetReefLevel();
    } else {
      m_superstructure.armStow();
      m_superstructure.elevatorStow();
      m_superstructure.wristStow();
    }
  }
}
