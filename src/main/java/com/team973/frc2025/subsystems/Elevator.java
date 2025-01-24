package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    TalonFXConfiguration leftMotorConfig = defaultElivatorMotorConfig();
    // looking at it from the front left is clockwise and right is counter clockwise
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motorLeft.setConfig(leftMotorConfig);
    TalonFXConfiguration rightMotorConfig = defaultElivatorMotorConfig();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorRight.setConfig(rightMotorConfig);
  }

  private static TalonFXConfiguration defaultElivatorMotorConfig() {
    TalonFXConfiguration defaultElivatorMotorConfig = new TalonFXConfiguration();
    defaultElivatorMotorConfig.Slot0.kS = 0.0;
    defaultElivatorMotorConfig.Slot0.kV = 0.15;
    defaultElivatorMotorConfig.Slot0.kA = 0.01;
    defaultElivatorMotorConfig.Slot0.kP = 4.0;
    defaultElivatorMotorConfig.Slot0.kI = 0.0;
    defaultElivatorMotorConfig.Slot0.kD = 0.0;
    defaultElivatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    defaultElivatorMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    defaultElivatorMotorConfig.MotionMagic.MotionMagicJerk = 160;
    // slot 1 is for velocity
    defaultElivatorMotorConfig.Slot1.kS = 0.0;
    defaultElivatorMotorConfig.Slot1.kV = 0.125 * 10.0 / 10.5;
    defaultElivatorMotorConfig.Slot1.kA = 0.0;
    defaultElivatorMotorConfig.Slot1.kP = 0.3;
    defaultElivatorMotorConfig.Slot1.kI = 0.0;
    defaultElivatorMotorConfig.Slot1.kD = 0.0;
    defaultElivatorMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
    defaultElivatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultElivatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    defaultElivatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultElivatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    defaultElivatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    return defaultElivatorMotorConfig;
  }

  public boolean motorAtTarget() {
    return (Math.abs(m_leftTargetPostion - m_motorLeft.getPosition().getValueAsDouble()) < 0.1
        && Math.abs(m_rightTargetPostion - m_motorRight.getPosition().getValueAsDouble()) < 0.1);
  }

  private void setTargetPostion(double targetPostion) {
    m_leftTargetPostion = targetPostion;
    m_rightTargetPostion = targetPostion;
  }

  private void setClosedLoop() {
    m_motorRight.setControl(ControlMode.MotionMagicVoltage, m_leftTargetPostion, 0);
    m_motorLeft.setControl(ControlMode.MotionMagicVoltage, m_rightTargetPostion, 0);
  }

  @Override
  public void update() {
    switch (m_mode) {
      case Level1:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
          setClosedLoop();
        }
        break;
      case Level2:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
          setClosedLoop();
        }
        break;
      case Level3:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
          setClosedLoop();
        }
        break;
      case Level4:
        if (m_lastMode != m_mode) {
          setTargetPostion(0.0);
          setClosedLoop();
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
