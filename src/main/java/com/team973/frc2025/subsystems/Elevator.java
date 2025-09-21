package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator implements Subsystem {
  private final RobotInfo.ElevatorInfo m_elevatorInfo;

  private final Logger m_logger;

  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;

  private ControlStatus m_controlStatus = ControlStatus.Off;

  private double m_targetPostionHeightinches;
  private double m_targetpositionLeway = 0.1;

  private int m_elevatorHomedCount = 0;

  private boolean m_lastHallSensorMode = true;

  private final DigitalInput m_hallSensor;

  private final SolidSignaler m_elevatorHomedBlinker =
      new SolidSignaler(
          RobotInfo.Colors.PURPLE,
          250,
          RobotInfo.SignalerInfo.ELEVATOR_HALL_SENSOR_SIGNALER_PRIORITY);

  private double m_levelOneOffset = 0.0;
  private double m_levelTwoOffset = 0.0;
  private double m_levelThreeOffset = 0.0;
  private double m_levelFourOffset = 0.0;

  private double m_netOffset = 0.0;
  private double m_algaeHighOffset = 0.0;
  private double m_algaeLowOffset = 0.0;
  private double m_algaeFloorOffset = 0.0;
  private CANdleManger m_candleManger;

  private boolean m_hallZeroingEnabled = true;

  public static enum ControlStatus {
    TargetPostion,
    Zero,
    Off,
  }

  private double heightInchesToMotorRotations(double postionHeight) {
    return postionHeight
        / m_elevatorInfo.MOTOR_GEAR_RATIO
        / 5.0
        / 36.0
        / Conversions.Distance.INCH_PER_MM;
  }

  private double motorRotationsToHeightInches(double motorPostion) {
    return motorPostion
        * m_elevatorInfo.MOTOR_GEAR_RATIO
        * 5.0
        * 36.0
        * Conversions.Distance.INCH_PER_MM;
  }

  public Elevator(Logger logger, CANdleManger candle, RobotInfo robotInfo) {
    m_logger = logger;
    m_candleManger = candle;
    m_elevatorInfo = robotInfo.ELEVATOR_INFO;
    m_candleManger.addSignaler(m_elevatorHomedBlinker);
    m_motorRight = new GreyTalonFX(21, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorRight"));
    m_motorLeft =
        new GreyTalonFX(20, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorLeft", 0.5));
    m_hallSensor = new DigitalInput(m_elevatorInfo.HALL_SENSOR_ID);

    TalonFXConfiguration leftMotorConfig = defaultElevatorMotorConfig();
    // looking at it from the front left is clockwise and right is counter clockwise
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motorLeft.setConfig(leftMotorConfig);
    TalonFXConfiguration rightMotorConfig = defaultElevatorMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorRight.setConfig(rightMotorConfig);
    m_motorRight.setPosition(0);

    m_motorLeft.setControl(new Follower(m_motorRight.getDeviceID(), true));
    m_motorRight.setPosition(0.0);
  }

  private TalonFXConfiguration defaultElevatorMotorConfig() {
    TalonFXConfiguration defaultElevatorMotorConfig = m_elevatorInfo.MOTOR_CONFIG.getConfig();
    // slot zero is for motion maginc
    defaultElevatorMotorConfig.Slot0.kS = m_elevatorInfo.ELEVATOR_MM_KS;
    defaultElevatorMotorConfig.Slot0.kV = m_elevatorInfo.ELEVATOR_MM_KV;
    defaultElevatorMotorConfig.Slot0.kA = m_elevatorInfo.ELEVATOR_MM_KA;
    defaultElevatorMotorConfig.Slot0.kP = m_elevatorInfo.ELEVATOR_MM_KP;
    defaultElevatorMotorConfig.Slot0.kI = m_elevatorInfo.ELEVATOR_MM_KI;
    defaultElevatorMotorConfig.Slot0.kD = m_elevatorInfo.ELEVATOR_MM_KD;
    // slot one is velocity
    defaultElevatorMotorConfig.Slot1.kS = m_elevatorInfo.ELEVATOR_V_KS;
    defaultElevatorMotorConfig.Slot1.kV = m_elevatorInfo.ELEVATOR_V_KV;
    defaultElevatorMotorConfig.Slot1.kA = m_elevatorInfo.ELEVATOR_V_KA;
    defaultElevatorMotorConfig.Slot1.kP = m_elevatorInfo.ELEVATOR_V_KP;
    defaultElevatorMotorConfig.Slot1.kI = m_elevatorInfo.ELEVATOR_V_KI;
    defaultElevatorMotorConfig.Slot1.kD = m_elevatorInfo.ELEVATOR_V_KD;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        m_elevatorInfo.ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicAcceleration =
        m_elevatorInfo.ELEVATOR_MOTION_MAGIC_ACCELERATION;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicJerk =
        m_elevatorInfo.ELEVATOR_MOTION_MAGIC_JERK;
    defaultElevatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.00;
    defaultElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return defaultElevatorMotorConfig;
  }

  public void setHallZeroingEnabled(boolean zeroingMode) {
    m_hallZeroingEnabled = zeroingMode;
  }

  private boolean hallsensor() {
    return !m_hallSensor.get();
  }

  private void maybeHomeElevator() {
    if (!m_lastHallSensorMode && hallsensor()) {
      m_motorRight.setPosition(
          heightInchesToMotorRotations(m_elevatorInfo.ELEVATOR_HOMING_POSTION_HEIGHT));
      m_elevatorHomedBlinker.enable();
      m_elevatorHomedCount++;
    }
    m_lastHallSensorMode = hallsensor();
  }

  public void home() {
    m_motorRight.setPosition(0);
    m_elevatorHomedBlinker.enable();
    m_elevatorHomedCount++;
  }

  public void setControlStatus(ControlStatus status) {
    m_controlStatus = status;
  }

  private boolean motorAtTarget(double motorRot) {
    return (Math.abs(m_targetPostionHeightinches - motorRotationsToHeightInches(motorRot))
        < m_targetpositionLeway);
  }

  public boolean motorAtTarget() {
    return motorAtTarget(m_motorRight.getPosition().getValueAsDouble());
  }

  public void setTargetPostion(double targetPostionHeightinches) {
    m_targetPostionHeightinches = targetPostionHeightinches;
    m_controlStatus = ControlStatus.TargetPostion;
  }

  public void incrementOffset(double offset, ReefLevel level) {
    switch (level) {
      case L_1:
        m_levelOneOffset += offset;
        break;
      case L_2:
        m_levelTwoOffset += offset;
        break;
      case L_3:
        m_levelThreeOffset += offset;
        break;
      case L_4:
        m_levelFourOffset += offset;
        break;
      case Net:
        m_netOffset += offset;
        break;
      case AlgaeHigh:
        m_algaeHighOffset += offset;
        break;
      case AlgaeLow:
        m_algaeLowOffset += offset;
        break;
      case AlgaeFloor:
        m_algaeFloorOffset += offset;
        break;
      case Processor:
        break;
      case Horizontal:
        break;
    }
  }

  public double getTargetPosition() {
    return m_targetPostionHeightinches;
  }

  public double getTargetPositionFromLevel(ReefLevel level) {
    switch (level) {
      case L_1:
        return m_elevatorInfo.LEVEL_1 + m_levelOneOffset;
      case L_2:
        return m_elevatorInfo.LEVEL_2 + m_levelTwoOffset;
      case L_3:
        return m_elevatorInfo.LEVEL_3 + m_levelThreeOffset;
      case L_4:
        return m_elevatorInfo.LEVEL_4 + m_levelFourOffset;
      case Net:
        return m_elevatorInfo.NET + m_netOffset;
      case AlgaeHigh:
        return m_elevatorInfo.ALGAE_HIGH + m_algaeHighOffset;
      case AlgaeLow:
        return m_elevatorInfo.ALGAE_LOW + m_algaeLowOffset;
      case AlgaeFloor:
        return m_elevatorInfo.ALGAE_FLOOR + m_algaeFloorOffset;
      case Processor:
        return m_elevatorInfo.ALGAE_STOW;
      case Horizontal:
        return m_elevatorInfo.CORAL_STOW;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  @Override
  public void update() {
    // switch (m_controlStatus) {
    //   case TargetPostion:
    //     m_motorRight.setControl(
    //         ControlMode.MotionMagicVoltage,
    //         heightInchesToMotorRotations(m_targetPostionHeightinches),
    //         0);
    //     break;
    //   case Zero:
    //     m_motorRight.setControl(ControlMode.DutyCycleOut, -0.1);
    //     break;
    //   case Off:
    //     m_motorRight.setControl(ControlMode.DutyCycleOut, 0, 0);
    //     break;
    // }
  }

  public double getHeightInchesFromMotorRot(double motorRot) {
    return motorRotationsToHeightInches(motorRot);
  }

  @Override
  public void log() {
    double motorRightRot = m_motorRight.getPosition().getValueAsDouble();

    m_motorLeft.log();
    m_motorRight.log();

    m_logger.log("currentPostionHeightInches", getHeightInchesFromMotorRot(motorRightRot));
    m_logger.log("targetPostionReached", motorAtTarget(motorRightRot));
    m_logger.log("targetPostionHeightInches", m_targetPostionHeightinches);

    m_logger.log("elevatorMode", m_controlStatus.toString());

    m_logger.log("hallSesnsorReturnElevator", hallsensor());

    m_logger.log("Level 1 Offset", m_levelOneOffset);
    m_logger.log("Level 2 Offset", m_levelTwoOffset);
    m_logger.log("Level 3 Offset", m_levelThreeOffset);
    m_logger.log("Level 4 Offset", m_levelFourOffset);
    m_logger.log("Algae Low Offset", m_algaeLowOffset);
    m_logger.log("Algae High Offset", m_algaeHighOffset);
    m_logger.log("Algae Floor Offset", m_algaeFloorOffset);
    m_logger.log("Net Offset", m_netOffset);

    m_logger.log("Elevator Homed Count", m_elevatorHomedCount);
  }

  @Override
  public void syncSensors() {
    if (m_hallZeroingEnabled) {
      maybeHomeElevator();
    }
  }

  @Override
  public void reset() {}
}
