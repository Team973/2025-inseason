package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Claw.ControlStatus;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Arm implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_armMotor;
  private ControlStatus m_mode = ControlStatus.Stow;
  private ControlStatus m_lastMode = ControlStatus.Stow;
  private double m_armTargetPostion;
  public static enum ControlStatus {
    High,
    Medium,
    Low,
    Stow,
  }

  public Arm(Logger logger) {
    m_logger = logger;
    m_armMotor = new GreyTalonFX(10, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("armMotor"));

    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.Slot0.kS = 0.0;
    armMotorConfig.Slot0.kV = 0.15;
    armMotorConfig.Slot0.kA = 0.01;
    armMotorConfig.Slot0.kP = 4.0;
    armMotorConfig.Slot0.kI = 0.0;
    armMotorConfig.Slot0.kD = 0.0;
    armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    armMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    armMotorConfig.MotionMagic.MotionMagicJerk = 160;
    armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_armMotor.setConfig(armMotorConfig);
  }

  public boolean motorAtTarget() {
    return (Math.abs(m_armTargetPostion - m_armMotor.getPosition().getValueAsDouble()) < 0.1);
  }

  @Override
  public void update() {
    switch (m_mode) {
      case High:
        if (m_lastMode != m_mode) {
          m_armTargetPostion = m_armMotor.getPosition().getValueAsDouble() + 0;
          m_armMotor.setControl(ControlMode.MotionMagicVoltage, m_armTargetPostion, 0);
        }
        break;
      case Medium:
        if (m_lastMode != m_mode) {
          m_armTargetPostion = m_armMotor.getPosition().getValueAsDouble() + 0;
          m_armMotor.setControl(ControlMode.MotionMagicVoltage, m_armTargetPostion, 0);
        }
        break;
      case Low:
        if (m_lastMode != m_mode) {
          m_armTargetPostion = m_armMotor.getPosition().getValueAsDouble() + 0;
          m_armMotor.setControl(ControlMode.MotionMagicVoltage, m_armTargetPostion, 0);
        }
        break;
      case Stow:
        m_armMotor.setControl(ControlMode.DutyCycleOut, 0, 1);
    }

    m_lastMode = m_mode;
  }

  @Override
  public void log() {}

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
