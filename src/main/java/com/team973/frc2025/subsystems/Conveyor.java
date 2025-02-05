package com.team973.frc2025.subsystems;

import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Conveyor implements Subsystem {
  private final Logger m_logger;

  public final GreyTalonFX m_conveyor;

  private ControlMode m_controlMode = ControlMode.ConveyorOff;

  public Conveyor(Logger logger) {
    m_logger = logger;
    m_conveyor = new GreyTalonFX(36, "Canivore", m_logger.subLogger("motor"));
  }

  public enum ControlMode {
    ConveyorForward,
    ConveyorBackward,
    ConveyorOff,
  }

  public void setControlMode(ControlMode mode) {
    m_controlMode = mode;
  }

  @Override
  public void log() {
    double conveyorPosition = m_conveyor.getPosition().getValueAsDouble();
    m_logger.log("CurrentPositionRotations", conveyorPosition);
    m_logger.log("CurrentMode", m_controlMode.toString());
    m_conveyor.log();
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    switch (m_controlMode) {
      case ConveyorForward:
        m_conveyor.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0.1, 0);
        break;
      case ConveyorBackward:
        m_conveyor.setControl(GreyTalonFX.ControlMode.DutyCycleOut, -0.1, 0);
        break;
      case ConveyorOff:
        m_conveyor.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  @Override
  public void reset() {}
}
