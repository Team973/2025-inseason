package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.SignalerInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Climb implements Subsystem {
  private static final double JOYSTICK_TO_MOTOR_ROTATIONS = 5.0;
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = JOYSTICK_TO_MOTOR_ROTATIONS * 20.0;

  public static final double HORIZONTAL_POSITION_DEG = 100.0;
  public static final double CLIMB_POSITION_DEG = 220.0;

  private static final double STOP_POSITION_DEG = 233.0;

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

    defaultMotorConfig.Slot0.kS = RobotInfo.ClimbInfo.CLIMB_KS;
    defaultMotorConfig.Slot0.kV = RobotInfo.ClimbInfo.CLIMB_KV;
    defaultMotorConfig.Slot0.kA = RobotInfo.ClimbInfo.CLIMB_KA;
    defaultMotorConfig.Slot0.kP = RobotInfo.ClimbInfo.CLIMB_KP;
    defaultMotorConfig.Slot0.kI = RobotInfo.ClimbInfo.CLIMB_KI;
    defaultMotorConfig.Slot0.kD = RobotInfo.ClimbInfo.CLIMB_KD;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        RobotInfo.ClimbInfo.CLIMB_MOTION_MAGIC_CRUISE_VELOCITY;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration =
        RobotInfo.ClimbInfo.CLIMB_MOTION_MAGIC_ACCELERATION;
    defaultMotorConfig.MotionMagic.MotionMagicJerk = RobotInfo.ClimbInfo.CLIMB_MOTION_MAGIC_JERK;
    // slot 1 is for velocity
    defaultMotorConfig.Slot1.kS = RobotInfo.ClimbInfo.CLIMB_KS;
    defaultMotorConfig.Slot1.kV = RobotInfo.ClimbInfo.CLIMB_KV;
    defaultMotorConfig.Slot1.kA = RobotInfo.ClimbInfo.CLIMB_KA;
    defaultMotorConfig.Slot1.kP = RobotInfo.ClimbInfo.CLIMB_KP;
    defaultMotorConfig.Slot1.kI = RobotInfo.ClimbInfo.CLIMB_KI;
    defaultMotorConfig.Slot1.kD = RobotInfo.ClimbInfo.CLIMB_KD;

    defaultMotorConfig.CurrentLimits.StatorCurrentLimit =
        RobotInfo.ClimbInfo.CLIMB_SATOR_CURRENT_LIMIT;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable =
        RobotInfo.ClimbInfo.CLIMB_SATOR_CURRENT_LIMIT_ENABLE;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit =
        RobotInfo.ClimbInfo.CLIMB_SUPPLY_CURRENT_LIMIT;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        RobotInfo.ClimbInfo.CLIMB_SUPPLY_CURRENT_LIMIT_ENABLE;
    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        RobotInfo.ClimbInfo.CLIMB_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;

    defaultMotorConfig.Voltage.PeakForwardVoltage = RobotInfo.ClimbInfo.CLIMB_PEAK_FORDWARD_VOLTAGE;
    defaultMotorConfig.Voltage.PeakReverseVoltage = RobotInfo.ClimbInfo.CLIMB_PEAK_REVERSE_VOLTAGE;

    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    defaultMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return defaultMotorConfig;
  }

  public void incrementTarget(double increment) {
    m_targetPosition =
        m_climb.getPosition().getValueAsDouble() + (increment * JOYSTICK_TO_MOTOR_ROTATIONS);
  }

  public double getClimbAngle() {
    return (m_climb.getPosition().getValueAsDouble() / RobotInfo.ClimbInfo.MOTOR_ROT_PER_CLIMB_ROT)
        * 360.0;
  }

  public void setTarget(double target) {
    m_targetPosition = (target / 360.0) * RobotInfo.ClimbInfo.MOTOR_ROT_PER_CLIMB_ROT;
  }

  @Override
  public void log() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();
    m_logger.log("CurrentPositionRotations", climbPosition);
    m_climb.log();
    m_logger.log("CurrentManualPower", m_manualPower);
    m_logger.log("Target Position", m_targetPosition);
    m_logger.log("Angle", getClimbAngle());
  }

  @Override
  public void syncSensors() {
    if (getClimbAngle() >= STOP_POSITION_DEG) {
      m_climbStopSignaler.enable();
    } else if (getClimbAngle() >= HORIZONTAL_POSITION_DEG) {
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
