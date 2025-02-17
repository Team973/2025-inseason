package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Elevator implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;
  private ControlStatus m_controlStatus = ControlStatus.Off;
  private double m_targetPostionHeightinches;
  private double m_targetpositionLeway = 0.1;
  double MOTOR_GEAR_RATIO = 10.0 / 56.0;
  private double m_manualPower;

  private double m_levelOneOffset = 0.0;
  private double m_levelTwoOffset = 0.0;
  private double m_levelThreeOffset = 0.0;
  private double m_levelFourOffset = 0.0;

  public static enum ControlStatus {
    Manual,
    TargetPostion,
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
    public static final double LEVEL_1 = 2.0;
    public static final double LEVEL_2 = 12.0;
    public static final double LEVEL_3 = 3.5;
    public static final double LEVEL_4 = 25.5;
    public static final double STOW = 0;
  }

  public Elevator(Logger logger) {
    m_logger = logger;
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
    defaultElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        50.0; // 32.0; // 64; // 32; // 16;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicAcceleration =
        300.0; // 40.0; // 500; // 40; // 20;
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

  public void incrementOffset(double offset, int level) {
    switch (level) {
      case 1:
        m_levelOneOffset += offset;
        break;
      case 2:
        m_levelTwoOffset += offset;
        break;
      case 3:
        m_levelThreeOffset += offset;
        break;
      case 4:
        m_levelFourOffset += offset;
        break;
    }
  }

  public double getTargetPositionFromLevel(int level) {
    switch (level) {
      case 1:
        return Presets.LEVEL_1 + m_levelOneOffset;
      case 2:
        return Presets.LEVEL_2 + m_levelTwoOffset;
      case 3:
        return Presets.LEVEL_3 + m_levelThreeOffset;
      case 4:
        return Presets.LEVEL_4 + m_levelFourOffset;
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
      case Off:
        m_motorRight.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  @Override
  public void log() {
    m_logger.log(
        "currentPostionHeightInches",
        motorRotationsToHeightInches(m_motorRight.getPosition().getValueAsDouble()));
    m_logger.log("targetPostionReached", motorAtTarget());
    m_logger.log("targetPostionHeightInches", m_targetPostionHeightinches);
    m_motorLeft.log();
    m_motorRight.log();
    m_logger.log("elevatorMode", m_controlStatus.toString());
    m_logger.log(
        "motorErrorInches",
        motorRotationsToHeightInches(m_motorRight.getClosedLoopError().getValueAsDouble()));
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
