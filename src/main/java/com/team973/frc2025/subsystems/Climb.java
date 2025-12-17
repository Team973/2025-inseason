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

public class Climb extends Subsystem.Stateless {
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
    defaultMotorConfig.Slot0.kS = m_climbInfo.CLIMB_MM_KS;
    defaultMotorConfig.Slot0.kV = m_climbInfo.CLIMB_MM_KV;
    defaultMotorConfig.Slot0.kA = m_climbInfo.CLIMB_MM_KA;
    defaultMotorConfig.Slot0.kP = m_climbInfo.CLIMB_MM_KP;
    defaultMotorConfig.Slot0.kI = m_climbInfo.CLIMB_MM_KI;
    defaultMotorConfig.Slot0.kD = m_climbInfo.CLIMB_MM_KD;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        m_climbInfo.MOTION_MAGIC_CRUISE_VELOCITY * 0.5;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration =
        m_climbInfo.MOTION_MAGIC_CRUISE_VELOCITY * 10.0;
    defaultMotorConfig.MotionMagic.MotionMagicJerk =
        m_climbInfo.MOTION_MAGIC_CRUISE_VELOCITY * 100.0;
    // slot 1 is for velocity
    defaultMotorConfig.Slot1.kS = m_climbInfo.CLIMB_V_KS;
    defaultMotorConfig.Slot1.kV = m_climbInfo.CLIMB_V_KV;
    defaultMotorConfig.Slot1.kA = m_climbInfo.CLIMB_V_KA;
    defaultMotorConfig.Slot1.kP = m_climbInfo.CLIMB_V_KP;
    defaultMotorConfig.Slot1.kI = m_climbInfo.CLIMB_V_KI;
    defaultMotorConfig.Slot1.kD = m_climbInfo.CLIMB_V_KD;

    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        m_climbInfo.CLIMB_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;

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
