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
  private ControlStatus m_mode = ControlStatus.Off;
  private double m_targetPostionHeightinches;
  private double m_targetpositionLeway = 0.1;
  double MOTOR_GEAR_RATIO = 10.0 / 56.0;
  private double m_manualPower;

  public static enum ControlStatus {
    Manual,
    TargetPostion,
    Off,
  }

  public void setmotorManualOutput(double joystick) {
    m_manualPower = joystick * 0.1;
    m_mode = ControlStatus.Manual;
  }

  private double heightInchesToMotorRotations(double postionHeight) {
    return postionHeight / MOTOR_GEAR_RATIO / 5.0 / 36.0 / Conversions.Distance.INCH_PER_MM;
  }

  private double motorRotationsToHeightInches(double motorPostion) {
    return motorPostion * MOTOR_GEAR_RATIO * 5.0 * 36.0 * Conversions.Distance.INCH_PER_MM;
  }

  public static class Presets {
    public static final double LEVEL_1 = 0.0;
    public static final double LEVEL_2 = 12.0;
    public static final double LEVEL_3 = 1.0;
    public static final double LEVEL_4 = 26.0;
    public static final double OFF = 0;
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
    defaultElevatorMotorConfig.Slot1.kP = 1.0;
    defaultElevatorMotorConfig.Slot1.kI = 0.0;
    defaultElevatorMotorConfig.Slot1.kD = 0.0;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 16;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 20;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicJerk = 160;
    defaultElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 15;
    defaultElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultElevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 10;
    defaultElevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultElevatorMotorConfig.Voltage.PeakForwardVoltage = 4;
    defaultElevatorMotorConfig.Voltage.PeakReverseVoltage = -4;
    defaultElevatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    defaultElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return defaultElevatorMotorConfig;
  }

  public void setModeOff() {
    m_mode = ControlStatus.Off;
  }

  public boolean motorAtTarget() {
    return (Math.abs(
            m_targetPostionHeightinches
                - motorRotationsToHeightInches(m_motorRight.getPosition().getValueAsDouble()))
        < m_targetpositionLeway);
  }

  public void setTargetPostion(double targetPostionHeightinches) {
    m_targetPostionHeightinches = targetPostionHeightinches;
    m_mode = ControlStatus.TargetPostion;
  }

  @Override
  public void update() {
    switch (m_mode) {
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
    m_logger.log("elevatorMode", m_mode.toString());
    m_logger.log(
        "motorErrorInches",
        motorRotationsToHeightInches(m_motorRight.getClosedLoopError().getValueAsDouble()));
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
