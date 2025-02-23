package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
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

  private final DigitalInput m_conveyorBackSensor;
  private final DigitalInput m_conveyorFrontSensor;
  private final CANrange m_clawAlgaeSensor;

  private ControlStatus m_mode = ControlStatus.Off;
  private ControlStatus m_lastMode = ControlStatus.Off;

  private final SolidSignaler m_clawHasPeiceSignaler =
      new SolidSignaler(
          RobotInfo.Colors.GREEN, 0, RobotInfo.SignalerInfo.PEICE_IN_CLAW_SIGNALER_PRIORTY);

  private double m_targetHoldPosition = 0;

  private double m_coralBackUpRot = 3.0;

  private CANdleManger m_caNdle;

  public static enum ControlStatus {
    IntakeCoral,
    IntakeAlgae,
    HoldCoral,
    ScoreCoral,
    ScoreAlgae,
    Off,
  }

  public Claw(Logger logger, CANdleManger candle) {
    m_logger = logger;
    m_caNdle = candle;

    m_caNdle.addSignaler(m_clawHasPeiceSignaler);
    m_clawMotor =
        new GreyTalonFX(
            ClawInfo.RIGHT_MOTOR_ID,
            RobotInfo.CANIVORE_CANBUS,
            m_logger.subLogger("clawMotor", 0.2));
    m_conveyor =
        new GreyTalonFX(
            ClawInfo.CONVEYOR_MOTOR_ID,
            RobotInfo.CANIVORE_CANBUS,
            m_logger.subLogger("conveyorMotor", 0.2));

    m_conveyorBackSensor = new DigitalInput(ClawInfo.CONVEYOR_BACK_SENSOR_ID);
    m_conveyorFrontSensor = new DigitalInput(ClawInfo.CONVEYOR_FRONT_SENSOR_ID);
    m_clawAlgaeSensor = new CANrange(ClawInfo.CLAW_ALGAE_CAN_ID, RobotInfo.CANIVORE_CANBUS);

    TalonFXConfiguration rightMotorConfig = defaultClawMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_clawMotor.setConfig(rightMotorConfig);

    TalonFXConfiguration conveyorConfig = defaultClawMotorConfig();

    conveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    conveyorConfig.Slot0.kP = 0.3;
    conveyorConfig.Slot0.kV = 0.1;

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
    return (Math.abs(m_targetHoldPosition - m_clawMotor.getPosition().getValueAsDouble()) < 0.1);
  }

  private boolean getConveyorBackSensor() {
    return m_conveyorBackSensor.get();
  }

  private boolean getConveyorFrontSensor() {
    return m_conveyorFrontSensor.get();
  }

  private boolean getClawAlgaeSensor() {
    return m_clawAlgaeSensor.getDistance().getValueAsDouble() < 0.1;
  }

  public boolean getSeesCoral() {
    return getConveyorFrontSensor() || getConveyorBackSensor();
  }

  public void coralScoredLED() {
    // TODO: Add back the algee sensor once tunned
    if (getSeesCoral()) {
      m_clawHasPeiceSignaler.enable();
    } else {
      m_clawHasPeiceSignaler.disable();
    }
  }

  @Override
  public void update() {
    switch (m_mode) {
      case IntakeCoral:
        if (getConveyorFrontSensor()) {
          // Too far forward --- back up!
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -10, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.VelocityVoltage, -10);
        } else if (getConveyorBackSensor() && !getConveyorFrontSensor()) {
          // Perfect spot!
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        } else {
          // Way too far back
          m_clawMotor.setControl(ControlMode.VelocityVoltage, 20, VELOCITY_VOLTAGE_PID_SLOT);
          m_conveyor.setControl(ControlMode.VelocityVoltage, 20);
        }
        break;
      case IntakeAlgae:
        if (getClawAlgaeSensor()) {
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

        if (m_lastMode != m_mode) {
          m_targetHoldPosition = m_clawMotor.getPosition().getValueAsDouble();
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

        m_clawMotor.setControl(ControlMode.VelocityVoltage, 50, VELOCITY_VOLTAGE_PID_SLOT);
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
    m_conveyor.log();

    m_logger.log("Conveyor Back Sensor", getConveyorBackSensor());
    m_logger.log("Conveyor Front Sensor", getConveyorFrontSensor());
    m_logger.log("Claw Algae Sensor", getClawAlgaeSensor());

    m_logger.log("target hold postion", m_targetHoldPosition);
    m_logger.log("target rotations hit", motorAtTarget());
    m_logger.log("mode", m_mode.toString());
    m_logger.log("AlgeeCANdistance", m_clawAlgaeSensor.getDistance().getValueAsDouble());
    m_logger.log("SensorSeesAlgee", getClawAlgaeSensor());

    SmartDashboard.putString("DB/String 4", "Coral Backup: " + m_coralBackUpRot);
  }

  @Override
  public void syncSensors() {
    coralScoredLED();
  }

  @Override
  public void reset() {}
}
