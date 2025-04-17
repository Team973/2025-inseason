package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.ClimbInfo;
import com.team973.frc2025.shared.RobotInfo.SignalerInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Climb implements Subsystem {
  private static final double JOYSTICK_TO_MOTOR_ROTATIONS = 5.0;
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = JOYSTICK_TO_MOTOR_ROTATIONS * 20.0;

  public static final double HORIZONTAL_POSITION_DEG = 100.0;
  public static final double CLIMB_POSITION_DEG = 263.0;

  private static final double STOP_POSITION_DEG = 263.0;

  private final Logger m_logger;

  private final GreyTalonFX m_climb;

  private final SolidSignaler m_climbHorizontalSignaler =
      new SolidSignaler(RobotInfo.Colors.HOT_PINK, 100.0, SignalerInfo.CLIMB_HORIZONTAL_PRIORITY);

  private final BlinkingSignaler m_climbStopSignaler =
      new BlinkingSignaler(
          RobotInfo.Colors.GREEN,
          RobotInfo.Colors.OFF,
          500.0,
          100.0,
          SignalerInfo.CLIMB_STOP_PRIORITY);

  private double m_manualPower = 0;
  private double m_targetPosition;

  public Climb(Logger logger, CANdleManger candle) {
    m_logger = logger;
    m_climb = new GreyTalonFX(23, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("climb motor"));
    m_climb.setConfig(defaultMotorConfig());

    m_climb.setPosition(0);
    m_targetPosition = 0;

    candle.addSignaler(m_climbHorizontalSignaler);
    candle.addSignaler(m_climbStopSignaler);
  }

  private static TalonFXConfiguration defaultMotorConfig() {
    TalonFXConfiguration defaultMotorConfig = new TalonFXConfiguration();
    defaultMotorConfig.Slot0.kS = 0.0;
    defaultMotorConfig.Slot0.kV = 0.15;
    defaultMotorConfig.Slot0.kA = 0.01;
    defaultMotorConfig.Slot0.kP = 6.4;
    defaultMotorConfig.Slot0.kI = 0.0;
    defaultMotorConfig.Slot0.kD = 0.04;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY * 0.5;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_CRUISE_VELOCITY * 10.0;
    defaultMotorConfig.MotionMagic.MotionMagicJerk = MOTION_MAGIC_CRUISE_VELOCITY * 100.0;
    // slot 1 is for velocity
    defaultMotorConfig.Slot1.kS = 0.0;
    defaultMotorConfig.Slot1.kV = 0;
    defaultMotorConfig.Slot1.kA = 0.0;
    defaultMotorConfig.Slot1.kP = 6.4;
    defaultMotorConfig.Slot1.kI = 0.0;
    defaultMotorConfig.Slot1.kD = 0.04;

    defaultMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    defaultMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    defaultMotorConfig.Voltage.PeakReverseVoltage = 0.0;

    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    defaultMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return defaultMotorConfig;
  }

  public void incrementTarget(double increment) {
    m_targetPosition =
        m_climb.getPosition().getValueAsDouble() + (increment * JOYSTICK_TO_MOTOR_ROTATIONS);
  }

  public double getClimbDegFromMotorRot(double rot) {
    return (rot / ClimbInfo.MOTOR_ROT_PER_CLIMB_ROT) * 360.0;
  }

  public double getClimbAngleDeg() {
    return getClimbDegFromMotorRot(m_climb.getPosition().getValueAsDouble());
  }

  public void setTargetAngleDeg(double target) {
    m_targetPosition = (target / 360.0) * ClimbInfo.MOTOR_ROT_PER_CLIMB_ROT;
  }

  @Override
  public void log() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();
    m_climb.log();
    m_logger.log("CurrentPositionRotations", climbPosition);
    m_logger.log("CurrentManualPower", m_manualPower);
    m_logger.log("Target Position Motor Rotations", m_targetPosition);
    m_logger.log("Target Position Deg", getClimbDegFromMotorRot(m_targetPosition));
    m_logger.log("Angle Deg", getClimbAngleDeg());
  }

  @Override
  public void syncSensors() {
    if (getClimbAngleDeg() >= STOP_POSITION_DEG) {
      m_climbStopSignaler.enable();
    } else if (getClimbAngleDeg() >= HORIZONTAL_POSITION_DEG) {
      m_climbHorizontalSignaler.enable();
    }
  }

  @Override
  public void update() {
    m_climb.setControl(GreyTalonFX.ControlMode.MotionMagicVoltage, m_targetPosition, 0);
  }

  @Override
  public void reset() {}
}
