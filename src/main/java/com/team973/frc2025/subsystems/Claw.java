package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Claw implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;
  private final DigitalInput m_sensor = new DigitalInput(0);
  private ControlStatus m_mode = ControlStatus.Stop;
  private ControlStatus m_lastMode = ControlStatus.Stop;
  private double m_leftTargetPostion = 0;
  private double m_rightTargetMotion = 0;

  public Claw(Logger logger) {
    m_logger = logger;
    m_motorRight = new GreyTalonFX(36, "Canivore", new Logger("shooterRight"));
    m_motorLeft = new GreyTalonFX(35, "Canivore", new Logger("shooterLeft"));

    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotorConfig.Slot0.kS = 0.0;
    rightMotorConfig.Slot0.kV = 0.0;
    rightMotorConfig.Slot0.kA = 0.0;
    rightMotorConfig.Slot0.kP = 1.0;
    rightMotorConfig.Slot0.kI = 0.0;
    rightMotorConfig.Slot0.kD = 0.0;
    rightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    rightMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    rightMotorConfig.MotionMagic.MotionMagicJerk = 160;
    m_motorRight.setConfig(rightMotorConfig);
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.Slot0.kS = 0.0;
    leftMotorConfig.Slot0.kV = 0.0;
    leftMotorConfig.Slot0.kA = 0.0;
    leftMotorConfig.Slot0.kP = 1.0;
    leftMotorConfig.Slot0.kI = 0.0;
    leftMotorConfig.Slot0.kD = 0.0;
    leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    leftMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    leftMotorConfig.MotionMagic.MotionMagicJerk = 160;
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorLeft.setConfig(leftMotorConfig);
  }

  public boolean motorAtTarget() {
    return (Math.abs(m_leftTargetPostion - m_motorLeft.getPosition().getValueAsDouble()) < 0.1
        && Math.abs(m_rightTargetMotion - m_motorRight.getPosition().getValueAsDouble()) < 0.1);
  }

  public boolean sensorSeeCoral() {
    return m_sensor.get();
  }

  public static enum ControlStatus {
    IntakeAndHold,
    Shoot,
    Stop,
    Retract,
    Score,
  }

  @Override
  public void update() {
    switch (m_mode) {
      case IntakeAndHold:
        if (sensorSeeCoral()) {
          m_motorRight.setControl(ControlMode.DutyCycleOut, 0);
          m_motorLeft.setControl(ControlMode.DutyCycleOut, 0);
        } else if (!sensorSeeCoral()) {
          m_motorRight.setControl(ControlMode.DutyCycleOut, 0.1);
          m_motorLeft.setControl(ControlMode.DutyCycleOut, 0.1);
        }
        break;
      case Shoot:
        m_motorRight.setControl(ControlMode.DutyCycleOut, 0.1);
        m_motorLeft.setControl(ControlMode.DutyCycleOut, 0.1);
        break;
      case Stop:
        m_motorRight.setControl(ControlMode.DutyCycleOut, 0);
        m_motorLeft.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case Score:
        if (m_lastMode != m_mode) {
          m_rightTargetMotion = m_motorRight.getPosition().getValueAsDouble() + 4.5;
          m_leftTargetPostion = m_motorLeft.getPosition().getValueAsDouble() + 4.5;
        }
        m_motorRight.setControl(ControlMode.MotionMagicVoltage, m_rightTargetMotion);
        m_motorLeft.setControl(ControlMode.MotionMagicVoltage, m_leftTargetPostion);
        break;
      case Retract:
        m_motorRight.setControl(ControlMode.DutyCycleOut, -0.1);
        m_motorLeft.setControl(ControlMode.DutyCycleOut, -0.1);
        break;
    }
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  @Override
  public void log() {
    m_logger.log("sensorSeeCoral", sensorSeeCoral());
    m_motorRight.log();
    m_motorLeft.log();
    m_logger.log("target postion left", m_leftTargetPostion);
    m_logger.log("target postion right", m_rightTargetMotion);
    m_logger.log("target rotations hit", motorAtTarget());
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
