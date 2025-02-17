package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.ClawInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw implements Subsystem {
  private static final int MOTION_MAGIC_PID_SLOT = 0;
  private static final int VELOCITY_VOLTAGE_PID_SLOT = 1;

  private final Logger m_logger;

  private final GreyTalonFX m_clawMotor;
  private final GreyTalonFX m_conveyor;

  private final DigitalInput m_backSensor;
  private final DigitalInput m_frontSensor;
  private final DigitalInput m_coralSensor;
  private final DigitalInput m_algaeSensor;

  private ControlStatus m_mode = ControlStatus.Off;
  private ControlStatus m_lastMode = ControlStatus.Off;

  public final SolidSignaler m_coralInclawBlinker = new SolidSignaler(RobotInfo.Colors.GREEN, 2);

  private double m_leftTargetPostion = 0;
  private double m_rightTargetPotion = 0;

  private double m_targetHoldPosition = 0;

  private double m_coralBackUpRot = 3.0;

  private boolean m_needsBackup = true;

  public static enum ControlStatus {
    IntakeCoral,
    IntakeAlgae,
    HoldCoral,
    ScoreCoral,
    ScoreAlgae,
    Off,
  }

  public Claw(Logger logger , CANdleManger candle) {
    m_logger = logger;


    candle.addSignaler(m_coralInclawBlinker);
    m_clawMotor =
        new GreyTalonFX(ClawInfo.RIGHT_MOTOR_ID, "Canivore", m_logger.subLogger("shooterRight"));
    m_conveyor =
        new GreyTalonFX(
            ClawInfo.CONVEYOR_MOTOR_ID, "Canivore", m_logger.subLogger("conveyor motor"));

    m_backSensor = new DigitalInput(ClawInfo.BACK_SENSOR_ID);
    m_frontSensor = new DigitalInput(ClawInfo.FRONT_SENSOR_ID);
    m_coralSensor = new DigitalInput(ClawInfo.CORAL_SENSOR_ID);
    m_algaeSensor = new DigitalInput(ClawInfo.ALGAE_SENSOR_ID);

    TalonFXConfiguration rightMotorConfig = defaultClawMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_clawMotor.setConfig(rightMotorConfig);

    TalonFXConfiguration conveyorConfig = defaultClawMotorConfig();

    conveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    conveyorConfig.Slot0.kP = 1.0;

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
    return (Math.abs(m_rightTargetPotion - m_clawMotor.getPosition().getValueAsDouble()) < 0.1);
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
    return !getFrontSensor() && getCoralSensor();
  }

  public boolean getSeesCoral() {
    return getFrontSensor() || getBackSensor() || getCoralSensor();
  }

  // public boolean getIsCoralInConveyor() {
  //   return getBackSensor() || getFrontSensor(); // || getCoralSensor();
  // }

  public void coralScoredLED() {
    if (getCoralSensor() ) {
      m_coralInclawBlinker.setEnabled(true);
    } else {
      m_coralInclawBlinker.setEnabled(false);
    }
  }

  @Override
  public void update() {
    switch (m_mode) {
      case IntakeCoral:
        if (!getBackSensor() && getFrontSensor()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.DutyCycleOut, 0);

          m_needsBackup = true;
        } else if (!getFrontSensor() && getCoralSensor()) {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -20, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.VelocityVoltage, -10);
        } else {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, 20, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.VelocityVoltage, 10);
        }
        break;
      case IntakeAlgae:
        if (getAlgaeSensor()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        } else {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -35, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.VelocityVoltage, -20);
        }
        break;
      case HoldCoral:
        // if (getFrontSensor()) { // if (!getCoralSensor() || getFrontSensor()) {
        //   m_clawMotor.setControl(ControlMode.VelocityVoltage, 15, VELOCITY_VOLTAGE_PID_SLOT);
        //   m_conveyor.setControl(ControlMode.VelocityVoltage, 7);
        // } else {

        if (m_lastMode != m_mode && m_needsBackup) {
          m_targetHoldPosition = m_clawMotor.getPosition().getValueAsDouble() - m_coralBackUpRot;
          m_needsBackup = false;
        }

        m_clawMotor.setControl(
            ControlMode.PositionVoltage, m_targetHoldPosition, MOTION_MAGIC_PID_SLOT);
        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        // }
        break;
      case ScoreCoral:
        // if (m_lastMode != m_mode) {
        //   m_rightTargetPotion = m_clawMotor.getPosition().getValueAsDouble() + 40;
        // }
        // m_clawMotor.setControl(
        //     ControlMode.MotionMagicVoltage, m_rightTargetPotion, MOTION_MAGIC_PID_SLOT);

        // if (motorAtTarget() && getCoralSensor()) {
        //   m_clawMotor.setControl(ControlMode.VelocityVoltage, 60, VELOCITY_VOLTAGE_PID_SLOT);
        // }

        m_clawMotor.setControl(ControlMode.VelocityVoltage, 35, VELOCITY_VOLTAGE_PID_SLOT);
        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case ScoreAlgae:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, -35, VELOCITY_VOLTAGE_PID_SLOT);
        m_conveyor.setControl(ControlMode.VelocityVoltage, -20);
        break;
      case Off:
        m_clawMotor.setControl(ControlMode.DutyCycleOut, 0, VELOCITY_VOLTAGE_PID_SLOT);
        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
    }
    m_lastMode = m_mode;
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  public void incrementBackup(double increment) {
    m_coralBackUpRot += increment;
  }

  @Override
  public void log() {
    m_clawMotor.log();

    m_logger.log("Back Sensor", getBackSensor());
    m_logger.log("Front Sensor", getFrontSensor());
    m_logger.log("Coral Sensor", getCoralSensor());
    m_logger.log("Algae Sensor", getAlgaeSensor());

    m_logger.log("target postion left", m_leftTargetPostion);
    m_logger.log("target postion right", m_rightTargetPotion);
    m_logger.log("target rotations hit", motorAtTarget());

    SmartDashboard.putString("DB/String 4", "Coral Backup: " + m_coralBackUpRot);
  }

  @Override
  public void syncSensors() {
    coralScoredLED();
  }

  @Override
  public void reset() {}
}
