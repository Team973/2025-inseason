package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Elevator implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;
  private ControlStatus m_mode = ControlStatus.Off;
  private ControlStatus m_lastMode = ControlStatus.Off;
  private double m_rightTargetPostion;
  private double m_leftTargetPostion;
  private double m_targetpositionLeway = 0.1;

  public static enum ControlStatus {
    Level1,
    Level2,
    Level3,
    Level4,
    Off,
  }

  public static class Presets {
    public static final double LEVEL_1 = 0;
    public static final double LEVEL_2 = 0;
    public static final double LEVEL_3 = 0;
    public static final double LEVEL_4 = 0;
    public static final double OFF = 0;
  }

  public Elevator(Logger logger) {
    m_logger = logger;
    m_motorRight =
        new GreyTalonFX(10, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorBottom"));
    m_motorLeft = new GreyTalonFX(50, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("motorTop"));

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
    defaultElevatorMotorConfig.Slot0.kS = 0.0;
    defaultElevatorMotorConfig.Slot0.kV = 0.15;
    defaultElevatorMotorConfig.Slot0.kA = 0.01;
    defaultElevatorMotorConfig.Slot0.kP = 4.0;
    defaultElevatorMotorConfig.Slot0.kI = 0.0;
    defaultElevatorMotorConfig.Slot0.kD = 0.0;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    defaultElevatorMotorConfig.MotionMagic.MotionMagicJerk = 160;
    // slot 1 is for velocity
    defaultElevatorMotorConfig.Slot1.kS = 0.0;
    defaultElevatorMotorConfig.Slot1.kV = 0.125 * 10.0 / 10.5;
    defaultElevatorMotorConfig.Slot1.kA = 0.0;
    defaultElevatorMotorConfig.Slot1.kP = 0.3;
    defaultElevatorMotorConfig.Slot1.kI = 0.0;
    defaultElevatorMotorConfig.Slot1.kD = 0.0;
    defaultElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 15;
    defaultElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultElevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 10;
    defaultElevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultElevatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    defaultElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    return defaultElevatorMotorConfig;
  }

  public boolean motorAtTarget() {
    return (Math.abs(m_leftTargetPostion - m_motorLeft.getPosition().getValueAsDouble())
            < m_targetpositionLeway
        && Math.abs(m_rightTargetPostion - m_motorRight.getPosition().getValueAsDouble())
            < m_targetpositionLeway);
  }

  private void setTargetPostion(double targetPostion) {
    m_leftTargetPostion = targetPostion;
    m_rightTargetPostion = targetPostion;
    m_motorRight.setControl(ControlMode.MotionMagicVoltage, m_leftTargetPostion, 0);
    m_motorLeft.setControl(ControlMode.MotionMagicVoltage, m_rightTargetPostion, 0);
  }

  @Override
  public void update() {
    switch (m_mode) {
      case Level1:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
        }
        break;
      case Level2:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
        }
        break;
      case Level3:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
        }
        break;
      case Level4:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
        }

        break;
      case Off:
        m_motorLeft.setControl(ControlMode.DutyCycleOut, 0, 0);
        m_motorRight.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
    m_lastMode = m_mode;
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  @Override
  public void log() {
    m_logger.log("target rotations hit", motorAtTarget());
    m_logger.log("target postion left", m_leftTargetPostion);
    m_logger.log("target postion right", m_rightTargetPostion);
    m_motorLeft.log();
    m_motorRight.log();
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
