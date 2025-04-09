package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
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
import java.util.Optional;

public class Claw implements Subsystem {
  private static final double ALGAE_SENSOR_STRENGTH_THRESHOLD = 1000.0;

  private final Logger m_logger;

  private final GreyTalonFX m_clawMotor;
  private final GreyTalonFX m_conveyor;

  private final DigitalInput m_backClawSensor;
  private final DigitalInput m_frontClawSensor;
  private final CANrange m_clawAlgaeSensor;

  private ControlStatus m_mode = ControlStatus.Off;

  private final SolidSignaler m_clawHasPeiceSignaler =
      new SolidSignaler(
          RobotInfo.Colors.GREEN, 0, RobotInfo.SignalerInfo.PEICE_IN_CLAW_SIGNALER_PRIORTY);

  private double m_targetHoldPosition = 0;
  private double m_coralBackUpRot = 3.0;
  private double m_filteredAlgaeDistMeters = 2.0;

  private CANdleManger m_caNdle;

  private double m_filteredAlgaeDistance;

  public static enum ControlStatus {
    IntakeCoral,
    IntakeAlgae,
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

    m_backClawSensor = new DigitalInput(ClawInfo.CONVEYOR_BACK_SENSOR_ID);
    m_frontClawSensor = new DigitalInput(ClawInfo.CONVEYOR_FRONT_SENSOR_ID);
    m_clawAlgaeSensor = new CANrange(ClawInfo.CLAW_ALGAE_CAN_ID, RobotInfo.CANIVORE_CANBUS);

    TalonFXConfiguration rightMotorConfig = defaultClawMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_clawMotor.setConfig(rightMotorConfig);

    TalonFXConfiguration conveyorConfig = defaultClawMotorConfig();

    conveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    conveyorConfig.Slot0.kP = 0.3;
    conveyorConfig.Slot0.kV = 0.1;

    m_conveyor.setConfig(conveyorConfig);

    CANrangeConfiguration algaeSensorConfig = new CANrangeConfiguration();
    m_clawAlgaeSensor.getConfigurator().apply(algaeSensorConfig);
  }

  public static TalonFXConfiguration defaultClawMotorConfig() {
    TalonFXConfiguration defaultMotorConfig = new TalonFXConfiguration();

    // Slot 0 PID constants are for velocity voltage
    defaultMotorConfig.Slot0.kS = RobotInfo.ClawInfo.CLAW_KS;
    defaultMotorConfig.Slot0.kV = RobotInfo.ClawInfo.CLAW_KV;
    defaultMotorConfig.Slot0.kA = RobotInfo.ClawInfo.CLAW_KA;
    defaultMotorConfig.Slot0.kP = RobotInfo.ClawInfo.CLAW_KP;
    defaultMotorConfig.Slot0.kI = RobotInfo.ClawInfo.CLAW_KI;
    defaultMotorConfig.Slot0.kD = RobotInfo.ClawInfo.CLAW_KD;

    defaultMotorConfig.CurrentLimits.StatorCurrentLimit =
        RobotInfo.ClawInfo.CLAW_SATOR_CURRENT_LIMIT;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable =
        RobotInfo.ClawInfo.CLAW_SATOR_CURRENT_LIMIT_ENABLE;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit =
        RobotInfo.ClawInfo.CLAW_SUPPLY_CURRENT_LIMIT;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        RobotInfo.ClawInfo.CLAW_SUPPLY_CURRENT_LIMIT_ENABLE;

    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        RobotInfo.ClawInfo.CLAW_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;

    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    return defaultMotorConfig;
  }

  public boolean motorAtTarget() {
    return (Math.abs(m_targetHoldPosition - m_clawMotor.getPosition().getValueAsDouble()) < 0.1);
  }

  private boolean getClawBackSensor() {
    return m_backClawSensor.get();
  }

  private boolean getClawFrontSensor() {
    return !m_frontClawSensor.get();
  }

  public boolean getSeesCoral() {
    return getClawFrontSensor() || getClawBackSensor();
  }

  private Optional<Double> getAlgaeDistance() {
    if (m_clawAlgaeSensor.getSignalStrength().getValueAsDouble()
        < ALGAE_SENSOR_STRENGTH_THRESHOLD) {
      return Optional.empty();
    }

    return Optional.of(m_clawAlgaeSensor.getDistance().getValueAsDouble());
  }

  public boolean getHasAlgae() {
    if (getAlgaeDistance().isEmpty()) {
      return false;
    }
    return m_filteredAlgaeDistMeters < 0.15;
  }

  public void coralScoredLED() {
    // TODO: Add back the algee sensor once tunned
    if (getSeesCoral() || getHasAlgae()) {
      m_clawHasPeiceSignaler.enable();
    } else {
      m_clawHasPeiceSignaler.disable();
    }
  }

  @Override
  public void update() {
    switch (m_mode) {
      case IntakeCoral:
        if (getClawFrontSensor()) {
          // Too far forward --- back up!
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -10);
          m_conveyor.setControl(ControlMode.VelocityVoltage, -10);
        } else if (getClawBackSensor() && !getClawFrontSensor()) {
          // Perfect spot!
          m_clawMotor.setControl(ControlMode.VelocityVoltage, 0);
          m_conveyor.setControl(ControlMode.VelocityVoltage, 0);
        } else {
          // Way too far back
          m_clawMotor.setControl(ControlMode.VelocityVoltage, 40);
          m_conveyor.setControl(ControlMode.VelocityVoltage, 80);
        }
        break;
      case IntakeAlgae:
        Optional<Double> algaeDistance = getAlgaeDistance();

        if (algaeDistance.isEmpty()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0);
        } else if (algaeDistance.get() > 0.3) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0);
          // } else if (algaeDistance.get() < 0.13) {
          //   m_clawMotor.setControl(ControlMode.VelocityVoltage, -4.0);
        } else {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -100.0);
        }

        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case ScoreCoral:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 50);
        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case ScoreAlgae:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 150);
        m_conveyor.setControl(ControlMode.VelocityVoltage, 0);
        break;
      case Off:
        m_clawMotor.setControl(ControlMode.DutyCycleOut, 0);
        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
    }
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

    m_logger.log("Filtered Algae Distance", m_filteredAlgaeDistance);
    m_logger.log("Has Algae", getHasAlgae());
    m_logger.log("Claw Motor Voltage", m_clawMotor.getMotorVoltage().getValueAsDouble());

    m_logger.log("Conveyor Back Sensor", getClawBackSensor());
    m_logger.log("Conveyor Front Sensor", getClawFrontSensor());

    m_logger.log("Algae Sensor Strength", m_clawAlgaeSensor.getSignalStrength().getValueAsDouble());
    m_logger.log(
        "Algae Sensor Health", m_clawAlgaeSensor.getMeasurementHealth().getValueAsDouble());

    m_logger.log("target hold postion", m_targetHoldPosition);
    m_logger.log("target rotations hit", motorAtTarget());
    m_logger.log("mode", m_mode.toString());
    m_logger.log("AlgeeCANdistance", m_clawAlgaeSensor.getDistance().getValueAsDouble());

    SmartDashboard.putString("DB/String 4", "Coral Backup: " + m_coralBackUpRot);
  }

  @Override
  public void syncSensors() {
    coralScoredLED();

    Optional<Double> algaeDist = getAlgaeDistance();

    if (algaeDist.isPresent()) {
      m_filteredAlgaeDistMeters = (m_filteredAlgaeDistMeters * 0.95) + (algaeDist.get() * 0.05);
    }
  }

  @Override
  public void reset() {}
}
