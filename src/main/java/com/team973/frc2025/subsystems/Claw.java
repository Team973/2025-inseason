package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private double m_rightTargetPotion = 0;

  public Claw(Logger logger) {
    m_logger = logger;
    m_motorRight = new GreyTalonFX(36, "Canivore", m_logger.subLogger("shooterRight"));
    m_motorLeft = new GreyTalonFX(35, "Canivore", m_logger.subLogger("shooterLeft"));

    TalonFXConfiguration rightMotorConfig = defaultClawMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motorRight.setConfig(rightMotorConfig);
    TalonFXConfiguration leftMotorConfig = defaultClawMotorConfig();
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorLeft.setConfig(leftMotorConfig);
  }

  public static TalonFXConfiguration defaultClawMotorConfig() {
    TalonFXConfiguration defaultMotorConfig = new TalonFXConfiguration();
    defaultMotorConfig.Slot0.kS = 0.0;
    defaultMotorConfig.Slot0.kV = 0.15;
    defaultMotorConfig.Slot0.kA = 0.01;
    defaultMotorConfig.Slot0.kP = 4.0;
    defaultMotorConfig.Slot0.kI = 0.0;
    defaultMotorConfig.Slot0.kD = 0.0;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    defaultMotorConfig.MotionMagic.MotionMagicJerk = 160;
    // slot 1 is for velocity
    defaultMotorConfig.Slot1.kS = 0.0;
    defaultMotorConfig.Slot1.kV = 0.125 * 10.0 / 10.5;
    defaultMotorConfig.Slot1.kA = 0.0;
    defaultMotorConfig.Slot1.kP = 0.3;
    defaultMotorConfig.Slot1.kI = 0.0;
    defaultMotorConfig.Slot1.kD = 0.0;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    return defaultMotorConfig;
  }

  public boolean motorAtTarget() {
    return (Math.abs(m_leftTargetPostion - m_motorLeft.getPosition().getValueAsDouble()) < 0.1
        && Math.abs(m_rightTargetPotion - m_motorRight.getPosition().getValueAsDouble()) < 0.1);
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

  private void setClosedLoopElivator(double motorDemandNum) {
    m_motorRight.setControl(ControlMode.VelocityVoltage, motorDemandNum, 1);
    m_motorLeft.setControl(ControlMode.VelocityVoltage, motorDemandNum, 1);
  }

  @Override
  public void update() {
    switch (m_mode) {
      case IntakeAndHold:
        if (sensorSeeCoral()) {
          setClosedLoopElivator(0);
        } else if (!sensorSeeCoral()) {
          setClosedLoopElivator(10);
        }
        break;
      case Shoot:
        setClosedLoopElivator(2);
        break;
      case Stop:
        setClosedLoopElivator(0);
        break;
      case Score:
        if (m_lastMode != m_mode) {
          m_rightTargetPotion = m_motorRight.getPosition().getValueAsDouble() + 4.5;
          m_leftTargetPostion = m_motorLeft.getPosition().getValueAsDouble() + 4.5;
        }
        m_motorRight.setControl(ControlMode.MotionMagicVoltage, m_rightTargetPotion, 0);
        m_motorLeft.setControl(ControlMode.MotionMagicVoltage, m_leftTargetPostion, 0);
        break;
      case Retract:
        setClosedLoopElivator(-5);
        break;
    }
    m_lastMode = m_mode;
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
    m_logger.log("target postion right", m_rightTargetPotion);
    m_logger.log("target rotations hit", motorAtTarget());
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
