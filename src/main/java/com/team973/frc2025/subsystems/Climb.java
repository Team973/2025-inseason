package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.Colors;
import com.team973.frc2025.shared.RobotInfo.SignalerInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Climb implements Subsystem {
  private final RobotInfo m_robotInfo;
  private final RobotInfo.ClimbInfo m_climbInfo;

  private final Logger m_logger;

  private final GreyTalonFX m_climb;

  private final SolidSignaler m_climbHorizontalSignaler =
      new SolidSignaler(RobotInfo.Colors.HOT_PINK, 100.0, SignalerInfo.CLIMB_HORIZONTAL_PRIORITY);

  private final BlinkingSignaler m_climbStopSignaler =
      new BlinkingSignaler(
          Colors.GREEN, Colors.OFF, 500.0, 100.0, SignalerInfo.CLIMB_STOP_PRIORITY);

  private double m_targetPosition;

  public Climb(Logger logger, CANdleManger candle) {
    m_robotInfo = RobotConfig.get();
    m_climbInfo = m_robotInfo.CLIMB_INFO;
    m_logger = logger;
    m_climb = new GreyTalonFX(23, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("climb motor"));
    m_climb.setConfig(defaultMotorConfig());

    m_climb.setPosition(0);
    m_targetPosition = 0;

    candle.addSignaler(m_climbHorizontalSignaler);
    candle.addSignaler(m_climbStopSignaler);
  }

  private TalonFXConfiguration defaultMotorConfig() {
    TalonFXConfiguration defaultMotorConfig = new TalonFXConfiguration();
    defaultMotorConfig.Slot0.kS = 0.0;
    defaultMotorConfig.Slot0.kV = 0.15;
    defaultMotorConfig.Slot0.kA = 0.01;
    defaultMotorConfig.Slot0.kP = 6.4;
    defaultMotorConfig.Slot0.kI = 0.0;
    defaultMotorConfig.Slot0.kD = 0.04;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        m_climbInfo.MOTION_MAGIC_CRUISE_VELOCITY * 0.5;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration =
        m_climbInfo.MOTION_MAGIC_CRUISE_VELOCITY * 10.0;
    defaultMotorConfig.MotionMagic.MotionMagicJerk =
        m_climbInfo.MOTION_MAGIC_CRUISE_VELOCITY * 100.0;
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
        m_climb.getPosition().getValueAsDouble()
            + (increment * m_climbInfo.JOYSTICK_TO_MOTOR_ROTATIONS);
  }

  public double getClimbDegFromMotorRot(double rot) {
    return (rot / m_climbInfo.MOTOR_ROT_PER_CLIMB_ROT) * 360.0;
  }

  public double getClimbAngleDegFromMotorRot(double motorRot) {
    return getClimbDegFromMotorRot(motorRot);
  }

  public void setTargetAngleDeg(double target) {
    m_targetPosition = (target / 360.0) * m_climbInfo.MOTOR_ROT_PER_CLIMB_ROT;
  }

  @Override
  public void log() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();
    m_climb.log();
    m_logger.log("CurrentPositionRotations", climbPosition);
    m_logger.log("Target Position Motor Rotations", m_targetPosition);
    m_logger.log("Target Position Deg", getClimbDegFromMotorRot(m_targetPosition));
    m_logger.log("Angle Deg", getClimbAngleDegFromMotorRot(climbPosition));
  }

  @Override
  public void syncSensors() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();

    if (getClimbAngleDegFromMotorRot(climbPosition) >= m_climbInfo.STOP_POSITION_DEG) {
      m_climbStopSignaler.enable();
    } else if (getClimbAngleDegFromMotorRot(climbPosition) >= m_climbInfo.HORIZONTAL_POSITION_DEG) {
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
