package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo.ClawInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Claw implements Subsystem {
  private static final int MOTION_MAGIC_PID_SLOT = 0;
  private static final int VELOCITY_VOLTAGE_PID_SLOT = 1;

  private final Logger m_logger;

  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;
  private final GreyTalonFX m_conveyor;

  private final DigitalInput m_backSensor;
  private final DigitalInput m_frontSensor;
  private final DigitalInput m_coralSensor;
  private final DigitalInput m_algaeSensor;

  private ControlStatus m_mode = ControlStatus.Off;
  private ControlStatus m_lastMode = ControlStatus.Off;

  private double m_leftTargetPostion = 0;
  private double m_rightTargetPotion = 0;

  public static enum ControlStatus {
    IntakeCoral,
    IntakeAlgae,
    HoldCoral,
    ScoreCoral,
    ScoreAlgae,
    Off,
  }

  public Claw(Logger logger) {
    m_logger = logger;

    m_motorRight =
        new GreyTalonFX(ClawInfo.RIGHT_MOTOR_ID, "Canivore", m_logger.subLogger("shooterRight"));
    m_motorLeft =
        new GreyTalonFX(ClawInfo.LEFT_MOTOR_ID, "Canivore", m_logger.subLogger("shooterLeft"));
    m_conveyor =
        new GreyTalonFX(
            ClawInfo.CONVEYOR_MOTOR_ID, "Canivore", m_logger.subLogger("conveyor motor"));

    m_backSensor = new DigitalInput(ClawInfo.BACK_SENSOR_ID);
    m_frontSensor = new DigitalInput(ClawInfo.FRONT_SENSOR_ID);
    m_coralSensor = new DigitalInput(ClawInfo.CORAL_SENSOR_ID);
    m_algaeSensor = new DigitalInput(ClawInfo.ALGAE_SENSOR_ID);

    TalonFXConfiguration rightMotorConfig = defaultClawMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motorRight.setConfig(rightMotorConfig);
    TalonFXConfiguration leftMotorConfig = defaultClawMotorConfig();
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorLeft.setConfig(leftMotorConfig);

    TalonFXConfiguration conveyorConfig = defaultClawMotorConfig();

    conveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_conveyor.setConfig(conveyorConfig);
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

  private boolean getBackSensor() {
    return m_backSensor.get();
  }

  private boolean getFrontSensor() {
    return m_frontSensor.get();
  }

  private boolean getCoralSensor() {
    return m_coralSensor.get();
  }

  private boolean getAlgaeSensor() {
    return m_algaeSensor.get();
  }

  public boolean getIsCoralInClaw() {
    return getCoralSensor() && !getFrontSensor();
  }

  public boolean getHasCoral() {
    return getBackSensor() || getFrontSensor() || getCoralSensor();
  }

  private void velocityRPS(double motorDemandNum) {
    m_motorRight.setControl(ControlMode.VelocityVoltage, motorDemandNum, VELOCITY_VOLTAGE_PID_SLOT);
    m_motorLeft.setControl(ControlMode.VelocityVoltage, motorDemandNum, VELOCITY_VOLTAGE_PID_SLOT);
    m_conveyor.setControl(ControlMode.VelocityVoltage, motorDemandNum);
  }

  @Override
  public void update() {
    switch (m_mode) {
      case IntakeCoral:
        if (!getBackSensor() && getFrontSensor()) {
          velocityRPS(0);
        } else if (!getFrontSensor() && getCoralSensor()) {
          velocityRPS(-10);
        } else {
          velocityRPS(10);
        }
        break;
      case IntakeAlgae:
        if (getAlgaeSensor()) {
          velocityRPS(0);
        } else {
          velocityRPS(-10);
        }
        break;
      case HoldCoral:
        if (!getCoralSensor() || getFrontSensor()) {
          velocityRPS(10);
        } else {
          velocityRPS(0);
        }
        break;
      case ScoreCoral:
        if (m_lastMode != m_mode) {
          m_rightTargetPotion = m_motorRight.getPosition().getValueAsDouble() + 4.5;
          m_leftTargetPostion = m_motorLeft.getPosition().getValueAsDouble() + 4.5;
        }
        m_motorRight.setControl(
            ControlMode.MotionMagicVoltage, m_rightTargetPotion, MOTION_MAGIC_PID_SLOT);
        m_motorLeft.setControl(
            ControlMode.MotionMagicVoltage, m_leftTargetPostion, MOTION_MAGIC_PID_SLOT);

        if (motorAtTarget() && getCoralSensor()) {
          m_motorRight.setControl(ControlMode.VelocityVoltage, 10, VELOCITY_VOLTAGE_PID_SLOT);
          m_motorLeft.setControl(ControlMode.VelocityVoltage, 10, VELOCITY_VOLTAGE_PID_SLOT);
        }

        m_conveyor.setControl(ControlMode.VelocityVoltage, 0);
        break;
      case ScoreAlgae:
        velocityRPS(-10);
        break;
      case Off:
        velocityRPS(0);
        break;
    }
    m_lastMode = m_mode;
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  @Override
  public void log() {
    m_motorRight.log();
    m_motorLeft.log();

    m_logger.log("Back Sensor", getBackSensor());
    m_logger.log("Front Sensor", getFrontSensor());
    m_logger.log("Coral Sensor", getCoralSensor());
    m_logger.log("Algae Sensor", getAlgaeSensor());

    m_logger.log("target postion left", m_leftTargetPostion);
    m_logger.log("target postion right", m_rightTargetPotion);
    m_logger.log("target rotations hit", motorAtTarget());
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
