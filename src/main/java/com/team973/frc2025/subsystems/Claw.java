package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.SignalerInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class Claw implements Subsystem {
  private final RobotInfo m_robotInfo;
  private final RobotInfo.ClawInfo m_clawInfo;
  private static final double ALGAE_SENSOR_STRENGTH_THRESHOLD = 2500.0;

  private final Logger m_logger;

  private final GreyTalonFX m_clawMotor;
  private final GreyTalonFX m_conveyor;

  private final DigitalInput m_backClawSensor;
  private final DigitalInput m_frontClawSensor;
  private final CANrange m_clawAlgaeSensor;

  private ControlStatus m_mode = ControlStatus.Off;

  private final SolidSignaler m_clawHasPeiceSignaler;

  private double m_targetHoldPosition = 0;
  private double m_coralBackUpRot = 3.0;
  private double m_filteredAlgaeDistMeters = 2.0;

  private CANdleManger m_caNdle;

  public static enum ControlStatus {
    IntakeCoral,
    IntakeAlgae,
    IntakeAlgaeFromFloor,
    ScoreCoral,
    ScoreCoralLevelOne,
    ScoreAlgae,
    ScoreAlgaeProcessor,
    Reverse,
    Off,
  }

  public Claw(Logger logger, CANdleManger candle, RobotInfo robotInfo) {
    m_logger = logger;
    m_caNdle = candle;
    m_robotInfo = robotInfo;
    m_clawInfo = robotInfo.new ClawInfo();

    m_clawHasPeiceSignaler = new SolidSignaler(
      RobotInfo.Colors.GREEN, 0, SignalerInfo.PEICE_IN_CLAW_SIGNALER_PRIORTY);

    m_caNdle.addSignaler(m_clawHasPeiceSignaler);
    m_clawMotor =
        new GreyTalonFX(
            m_clawInfo.RIGHT_MOTOR_ID,
            m_robotInfo.CANIVORE_CANBUS,
            m_logger.subLogger("clawMotor", 0.2));
    m_conveyor =
        new GreyTalonFX(
            m_clawInfo.CONVEYOR_MOTOR_ID,
            m_robotInfo.CANIVORE_CANBUS,
            m_logger.subLogger("conveyorMotor", 0.2));

    m_backClawSensor = new DigitalInput(m_clawInfo.CONVEYOR_BACK_SENSOR_ID);
    m_frontClawSensor = new DigitalInput(m_clawInfo.CONVEYOR_FRONT_SENSOR_ID);
    m_clawAlgaeSensor = new CANrange(m_clawInfo.CLAW_ALGAE_CAN_ID, m_robotInfo.CANIVORE_CANBUS);

    TalonFXConfiguration rightMotorConfig = defaultClawMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
    defaultMotorConfig.Slot0.kS = 0.0;
    defaultMotorConfig.Slot0.kV = 0.125 * 10.0 / 10.5;
    defaultMotorConfig.Slot0.kA = 0.0;
    defaultMotorConfig.Slot0.kP = 0.3;
    defaultMotorConfig.Slot0.kI = 0.0;
    defaultMotorConfig.Slot0.kD = 0.0;

    defaultMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    defaultMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    defaultMotorConfig.Voltage.PeakReverseVoltage = -12.0;

    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

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
    // TODO: Add back the algae sensor once tunned
    if (getSeesCoral() || getHasAlgae()) {
      m_clawHasPeiceSignaler.enable();
    } else {
      m_clawHasPeiceSignaler.disable();
    }
  }

  @Override
  public void update() {
    Optional<Double> algaeDistance = getAlgaeDistance();

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
          m_clawMotor.setControl(ControlMode.VelocityVoltage, 90);
          m_conveyor.setControl(ControlMode.VelocityVoltage, 180);
        }
        break;
      case IntakeAlgae:
        if (algaeDistance.isEmpty()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0);
        } else if (algaeDistance.get() > 0.3) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0);
        } else if (algaeDistance.get() < 0.13) {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -10.0);
        } else {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -100.0);
        }

        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case IntakeAlgaeFromFloor:
        if (algaeDistance.isEmpty()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, -100.0);
        } else if (algaeDistance.get() > 0.3) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, -100.0);
        } else if (algaeDistance.get() < 0.13) {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -10.0);
        } else {
          m_clawMotor.setControl(ControlMode.VelocityVoltage, -100.0);
        }

        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case ScoreCoral:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 100);
        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case ScoreCoralLevelOne:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 50);
        m_conveyor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case ScoreAlgae:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 450);
        m_conveyor.setControl(ControlMode.VelocityVoltage, 0);
        break;
      case ScoreAlgaeProcessor:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 50);
        m_conveyor.setControl(ControlMode.VelocityVoltage, 0);
        break;
      case Reverse:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, -20);
        m_conveyor.setControl(ControlMode.VelocityVoltage, -40);
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

    m_logger.log("Filtered Algae Distance Meters", m_filteredAlgaeDistMeters);
    m_logger.log("Has Algae", getHasAlgae());

    m_logger.log("Conveyor Back Sensor", getClawBackSensor());
    m_logger.log("Conveyor Front Sensor", getClawFrontSensor());

    m_logger.log("Algae Sensor Strength", m_clawAlgaeSensor.getSignalStrength().getValueAsDouble());

    m_logger.log("target hold postion", m_targetHoldPosition);
    m_logger.log("target rotations hit", motorAtTarget());
    m_logger.log("mode", m_mode.toString());
    m_logger.log("AlgaeCANdistance", m_clawAlgaeSensor.getDistance().getValueAsDouble());

    SmartDashboard.putString("DB/String 4", "Coral Backup: " + m_coralBackUpRot);
  }

  @Override
  public void syncSensors() {
    coralScoredLED();

    Optional<Double> algaeDist = getAlgaeDistance();

    if (algaeDist.isPresent()) {
      m_filteredAlgaeDistMeters = (m_filteredAlgaeDistMeters * 0.9) + (algaeDist.get() * 0.1);
    }
  }

  @Override
  public void reset() {}
}
