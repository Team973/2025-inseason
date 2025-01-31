package com.team973.frc2025.subsystems;

import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climb implements Subsystem {

  private final DigitalInput m_bannerSensor = new DigitalInput(2);

  private boolean bannerSensorSeesCoral() {
    return m_bannerSensor.get();
  }

  private final Logger m_logger;

  public final GreyTalonFX m_climb;

  private ControlMode m_controlMode = ControlMode.Stow;

  public Climb(Logger logger) {
    m_logger = logger;
    m_climb = new GreyTalonFX(35, "Canivore", m_logger);
  }

  public enum ControlMode {
    ClimbLow,
    ClimbHigh,
    Stow,
  }

  public void setControlMode(ControlMode mode) {
    m_controlMode = mode;
  }

  @Override
  public void log() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();
    m_logger.log("Position", climbPosition);
    m_logger.log("SeesCoral", bannerSensorSeesCoral());
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    switch (m_controlMode) {
      case Stow:
        m_climb.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0);
        break;
      case ClimbLow:
        m_climb.setControl(GreyTalonFX.ControlMode.DutyCycleOut, -0.1);
        break;
      case ClimbHigh:
        m_climb.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0.1);
        break;
    }
  }

  @Override
  public void reset() {}
}
