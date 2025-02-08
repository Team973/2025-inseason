package com.team973.frc2025.subsystems;

import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure implements Subsystem {
  private final Claw m_claw;
  private final Climb m_climb;
  private final DriveController m_driveController;

  private State m_state = State.Off;
  private State m_lastState = State.Off;

  private int m_targetReefLevel = 1;

  private boolean m_manualScore = false;

  public enum State {
    IntakeCoral,
    IntakeAlgae,
    ScoreCoral,
    ScoreAlgae,
    Climb,
    Manual,
    Off
  }

  public Superstructure(Claw claw, Climb climb, DriveController driveController) {
    m_claw = claw;
    m_climb = climb;
    m_driveController = driveController;
  }

  public void setState(State state) {
    m_state = state;
  }

  public void setManualScore(boolean score) {
    m_manualScore = score;
  }

  public void incrementTargetReefLevel(int increment) {
    m_targetReefLevel += increment;

    if (m_targetReefLevel > 4) {
      m_targetReefLevel = 1;
    } else if (m_targetReefLevel < 1) {
      m_targetReefLevel = 4;
    }
  }

  public boolean finishedScoring() {
    return !m_claw.getHasCoral();
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
        if (finishedScoring()
            && m_driveController.getDriveWithLimelight().reachedTargetInitialPose()) {
          m_claw.setControl(Claw.ControlStatus.IntakeCoral);
        } else if (finishedScoring()) {
          m_claw.setControl(Claw.ControlStatus.Off);
        } else if (m_driveController.getDriveWithLimelight().reachedTargetFinalPose()) {
          m_claw.setControl(Claw.ControlStatus.ScoreCoral);
        } else {
          m_claw.setControl(Claw.ControlStatus.HoldCoral);
        }

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
      case Manual:
        if (m_lastState != m_state) {
          m_manualScore = false;
        }

        if (m_manualScore) {
          m_claw.setControl(Claw.ControlStatus.ScoreCoral);
        } else {
          m_claw.setControl(Claw.ControlStatus.HoldCoral);
        }

        m_climb.setControlMode(Climb.ControlMode.OffState);
        break;
      case Off:
        m_claw.setControl(Claw.ControlStatus.Off);
        m_climb.setControlMode(Climb.ControlMode.OffState);
        break;
    }

    m_lastState = m_state;

    m_claw.update();
    m_climb.log();
  }

  public void reset() {
    m_claw.reset();
    m_climb.log();
  }
}
