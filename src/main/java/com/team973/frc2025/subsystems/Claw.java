package com.team973.frc2025.subsystems;

import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;

public class Claw {
  private final Logger m_logger;
  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;

  public Claw(Logger logger) {
    m_logger = logger;
    m_motorRight = new GreyTalonFX(36, "Canivore", new Logger("shooterRight"));
    m_motorLeft = new GreyTalonFX(35, "Canivore", new Logger("shooterLeft"));
  }

  public void shoot() {
    m_motorRight.setControl(ControlMode.DutyCycleOut, -0.1);
    m_motorLeft.setControl(ControlMode.DutyCycleOut, 0.1);
  }

  public void retract() {
    m_motorRight.setControl(ControlMode.DutyCycleOut, 0.1);
    m_motorLeft.setControl(ControlMode.DutyCycleOut, -0.1);
  }

  public void stop() {
    m_motorRight.setControl(ControlMode.DutyCycleOut, 0);
    m_motorLeft.setControl(ControlMode.DutyCycleOut, 0);
  }
}
