package com.team973.frc2025.subsystems;

import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;

public class Claw {
  private final Logger m_logger;
  private final GreyTalonFX m_shooterRight;
  private final GreyTalonFX m_shooterLeft;

  public Claw(Logger logger) {
    m_logger = logger;
    m_shooterRight = new GreyTalonFX(36, "Canivore", new Logger("shooterRight"));
    m_shooterLeft = new GreyTalonFX(35, "Canivore", new Logger("shooterLeft"));
  }

  public void shoot() {
    m_shooterRight.setControl(ControlMode.DutyCycleOut, -0.1);
    m_shooterLeft.setControl(ControlMode.DutyCycleOut, 0.1);
  }

  public void retract() {
    m_shooterRight.setControl(ControlMode.DutyCycleOut, 0.1);
    m_shooterLeft.setControl(ControlMode.DutyCycleOut, -0.1);
  }

  public void stop() {
    m_shooterRight.setControl(ControlMode.DutyCycleOut, 0);
    m_shooterLeft.setControl(ControlMode.DutyCycleOut, 0);
  }
}
