package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;
  private ControlStatus m_controlStatus = ControlStatus.Off;
  private double m_targetPostionHeightinches;
  private double m_targetpositionLeway = 0.1;
  double MOTOR_GEAR_RATIO = 10.0 / 56.0;
  private double m_manualPower;
  private boolean m_lastHallSensorMode = true;
  private final DigitalInput m_hallSensor = new DigitalInput(RobotInfo.ElevatorInfo.HALL_SENSOR_ID);

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

  private double ELEVATOR_HOMING_POSTION_HEIGHT = 0.25;
  private CANdleManger m_candleManger;

  public static enum ControlStatus {
    Manual,
    TargetPostion,
    Zero,
    Off,
  }

  public void setmotorManualOutput(double joystick) {
    m_manualPower = joystick * 0.1;
    m_controlStatus = ControlStatus.Manual;
  }

  private double heightInchesToMotorRotations(double postionHeight) {
    return postionHeight / MOTOR_GEAR_RATIO / 5.0 / 36.0 / Conversions.Distance.INCH_PER_MM;
  }

  private double motorRotationsToHeightInches(double motorPostion) {
    return motorPostion * MOTOR_GEAR_RATIO * 5.0 * 36.0 * Conversions.Distance.INCH_PER_MM;
  }

  public static class Presets {
    private static final double LEVEL_1 = 0.0;
    private static final double LEVEL_2 = 17.0;
    private static final double LEVEL_3 = 2.5;
    private static final double LEVEL_4 = 27.0;
    public static final double CORAL_STOW = 0.5;

    private static final double NET = 27.0;
    private static final double ALGAE_HIGH = 5.0;
    private static final double ALGAE_LOW = 19.0;
    private static final double ALGAE_FLOOR = 0.0;
    public static final double ALGAE_STOW = 1.0;
  }

  public Elevator(Logger logger, CANdleManger candle) {
    m_logger = logger;
    m_candleManger = candle;
    m_candleManger.addSignaler(m_elevatorHomedBlinker);
    m_motorRight = new GreyTalonFX(21, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorRight"));
    m_motorLeft = new GreyTalonFX(20, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorLeft"));

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

  private static TalonFXConfiguration defaultElevatorMotorConfig() {
    TalonFXConfiguration defaultElevatorMotorConfig = new TalonFXConfiguration();
    // slot zero is for motion maginc
    defaultElevatorMotorConfig.Slot0.kS = 0.0;
    defaultElevatorMotorConfig.Slot0.kV = 0.15;
    defaultElevatorMotorConfig.Slot0.kA = 0.01;
    defaultElevatorMotorConfig.Slot0.kP = 4.0;
    defaultElevatorMotorConfig.Slot0.kI = 0.0;
    defaultElevatorMotorConfig.Slot0.kD = 0.0;
    // slot one is velocity
    defaultElevatorMotorConfig.Slot1.kS = 0.0;
    defaultElevatorMotorConfig.Slot1.kV = 0.0;
    defaultElevatorMotorConfig.Slot1.kA = 0.0;
    defaultElevatorMotorConfig.Slot1.kP = 125.0;
    defaultElevatorMotorConfig.Slot1.kI = 0.0;
    defaultElevatorMotorConfig.Slot1.kD = 0.0;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 65.0;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 400.0;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicJerk = 2000.0;
    defaultElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
    defaultElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultElevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    defaultElevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultElevatorMotorConfig.Voltage.PeakForwardVoltage = 12;
    defaultElevatorMotorConfig.Voltage.PeakReverseVoltage = -12;
    defaultElevatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.00;
    defaultElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return defaultElevatorMotorConfig;
  }

  private boolean hallsensor() {
    return !m_hallSensor.get();
  }

  private void maybeHomeElevator() {
    if (!m_lastHallSensorMode && hallsensor()) {
      m_motorRight.setPosition(heightInchesToMotorRotations(ELEVATOR_HOMING_POSTION_HEIGHT));
      m_elevatorHomedBlinker.enable();
    }
    m_lastHallSensorMode = hallsensor();
  }

  public void setControlStatus(ControlStatus status) {
    m_controlStatus = status;
  }

  public boolean motorAtTarget() {
    return (Math.abs(
            m_targetPostionHeightinches
                - motorRotationsToHeightInches(m_motorRight.getPosition().getValueAsDouble()))
        < m_targetpositionLeway);
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
        return Presets.LEVEL_1 + m_levelOneOffset;
      case L_2:
        return Presets.LEVEL_2 + m_levelTwoOffset;
      case L_3:
        return Presets.LEVEL_3 + m_levelThreeOffset;
      case L_4:
        return Presets.LEVEL_4 + m_levelFourOffset;
      case Net:
        return Presets.NET + m_netOffset;
      case AlgaeHigh:
        return Presets.ALGAE_HIGH + m_algaeHighOffset;
      case AlgaeLow:
        return Presets.ALGAE_LOW + m_algaeLowOffset;
      case AlgaeFloor:
        return Presets.ALGAE_FLOOR + m_algaeFloorOffset;
      case Horizontal:
        return Presets.CORAL_STOW;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  @Override
  public void update() {
    switch (m_controlStatus) {
      case Manual:
        m_motorRight.setControl(ControlMode.DutyCycleOut, m_manualPower, 0);
        break;
      case TargetPostion:
        m_motorRight.setControl(
            ControlMode.MotionMagicVoltage,
            heightInchesToMotorRotations(m_targetPostionHeightinches),
            0);
        break;
      case Zero:
        m_motorRight.setControl(ControlMode.DutyCycleOut, -0.1);
        break;
      case Off:
        m_motorRight.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  public double getHeightInches() {
    return motorRotationsToHeightInches(m_motorRight.getPosition().getValueAsDouble());
  }

  @Override
  public void log() {
    m_logger.log("currentPostionHeightInches", getHeightInches());
    m_logger.log("targetPostionReached", motorAtTarget());
    m_logger.log("targetPostionHeightInches", m_targetPostionHeightinches);
    m_motorLeft.log();
    m_motorRight.log();
    m_logger.log("elevatorMode", m_controlStatus.toString());
    m_logger.log(
        "motorErrorInches",
        motorRotationsToHeightInches(m_motorRight.getClosedLoopError().getValueAsDouble()));
    m_logger.log("hallSesnsorReturnElevator", hallsensor());

    m_logger.log("Level 1 Offset", m_levelOneOffset);
    m_logger.log("Level 2 Offset", m_levelTwoOffset);
    m_logger.log("Level 3 Offset", m_levelThreeOffset);
    m_logger.log("Level 4 Offset", m_levelFourOffset);
    m_logger.log("Algae Low Offset", m_algaeLowOffset);
    m_logger.log("Algae High Offset", m_algaeHighOffset);
    m_logger.log("Algae Floor Offset", m_algaeFloorOffset);
  }

  @Override
  public void syncSensors() {
    maybeHomeElevator();
  }

  @Override
  public void reset() {}
}
