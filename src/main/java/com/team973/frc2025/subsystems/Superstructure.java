package com.team973.frc2025.subsystems;

import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure implements Subsystem {
  private final Claw m_claw;
  private final Climb m_climb;

  private State m_state = State.Off;

  private int m_targetReefLevel = 1;

  public enum State {
    IntakeCoral,
    IntakeAlgae,
    ScoreCoral,
    ScoreAlgae,
    Climb,
    Off
  }

  public Superstructure(Claw claw, Climb climb) {
    m_claw = claw;
    m_climb = climb;
  }

  public void setState(State state) {
    m_state = state;
  }

  public void incrementTargetReefLevel(int increment) {
    m_targetReefLevel += increment;

    if (m_targetReefLevel > 4) {
      m_targetReefLevel = 1;
    } else if (m_targetReefLevel < 1) {
      m_targetReefLevel = 4;
    }
  }

  public boolean readyToScore() {
    return true; // TODO: check armivator position
  }

  public void log() {
    SmartDashboard.putString("DB/String 1", "Reef Level: " + m_targetReefLevel);

    m_claw.log();
    m_climb.log();
  }

  public void syncSensors() {
    m_claw.syncSensors();
    m_climb.log();
  }

  public void update() {
    switch (m_state) {
      case IntakeCoral:
        m_claw.setControl(Claw.ControlStatus.IntakeCoral);
        m_climb.setControlMode(Climb.ControlMode.OffState);
        break;
      case IntakeAlgae:
        m_claw.setControl(Claw.ControlStatus.IntakeAlgae);
        m_climb.setControlMode(Climb.ControlMode.OffState);
        break;
      case ScoreCoral:
        m_claw.setControl(Claw.ControlStatus.ScoreCoral);
        m_climb.setControlMode(Climb.ControlMode.OffState);
        break;
      case ScoreAlgae:
        m_claw.setControl(Claw.ControlStatus.ScoreAlgae);
        m_climb.setControlMode(Climb.ControlMode.OffState);
        break;
      case Climb:
        m_claw.setControl(Claw.ControlStatus.Off);
        m_climb.setControlMode(Climb.ControlMode.ClimbLow);
        break;
      case Off:
        m_claw.setControl(Claw.ControlStatus.Off);
        m_climb.setControlMode(Climb.ControlMode.OffState);
        break;
    }

    m_claw.update();
    m_climb.log();
  }

  public void reset() {
    m_claw.reset();
    m_climb.log();
  }
}
