package com.team973.frc2025.subsystems;

import com.team973.lib.util.Subsystem;

public class Superstructure implements Subsystem {
  private final Claw m_claw;
  private final Conveyor m_conveyor;
  private final Climb m_climb;

  private State m_state = State.Off;

  public enum State {
    Intake,
    Score,
    Climb,
    Off
  }

  public Superstructure(Claw claw, Conveyor conveyor, Climb climb) {
    m_claw = claw;
    m_conveyor = conveyor;
    m_climb = climb;
  }

  public void setState(State state) {
    m_state = state;
  }

  public void log() {
    m_claw.log();
    m_conveyor.log();
    m_climb.log();
  }

  public void syncSensors() {
    m_claw.syncSensors();
    m_conveyor.syncSensors();
    m_climb.log();
  }

  public void update() {
    switch (m_state) {
      case Intake:
        break;
      case Score:
        break;
      case Climb:
        break;
      case Off:
        break;
    }

    m_claw.update();
    m_conveyor.update();
    m_climb.log();
  }

  public void reset() {
    m_claw.reset();
    m_conveyor.reset();
    m_climb.log();
  }
}
