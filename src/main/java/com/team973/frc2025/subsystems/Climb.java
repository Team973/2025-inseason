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
import edu.wpi.first.wpilibj.DigitalInput;

public class Climb implements Subsystem {
  private static final double JOYSTICK_TO_MOTOR_ROTATIONS = 15.0;
  private static final double MOTION_MAGIC_CRUISE_VELOCITY = JOYSTICK_TO_MOTOR_ROTATIONS * 20.0;

  private final DigitalInput m_bannerSensor = new DigitalInput(9);

  private boolean bannerSensorSeesCoral() {
    return m_bannerSensor.get();
  }

  private final Logger m_logger;

  private final GreyTalonFX m_climb;

  private final SolidSignaler m_climbHorizontalSignaler =
      new SolidSignaler(RobotInfo.Colors.PINK, 100.0, SignalerInfo.CLIMB_HORIZONTAL_PRIORITY);
  private final SolidSignaler m_climbStopSignaler =
      new SolidSignaler(RobotInfo.Colors.ORANGE, 100.0, SignalerInfo.CLIMB_STOP_PRIORITY);

  private double m_manualPower = 0;
  private double m_targetPosition;

  private ControlMode m_controlMode = ControlMode.ClimbLow;

  public Climb(Logger logger, CANdleManger candle) {
    m_logger = logger;
    m_climb = new GreyTalonFX(23, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("climb motor"));
    m_climb.setConfig(defaultMotorConfig());

    m_targetPosition = m_climb.getPosition().getValueAsDouble();

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
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_CRUISE_VELOCITY * 10.0;
    defaultMotorConfig.MotionMagic.MotionMagicJerk = 1600;
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

    defaultMotorConfig.Voltage.PeakForwardVoltage = 2.0;
    defaultMotorConfig.Voltage.PeakReverseVoltage = -2.0;

    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    defaultMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    return defaultMotorConfig;
  }

  public enum ControlMode {
    ClimbLow,
    ClimbStow,
    OffState,
    JoystickMode,
  }

  public void setControlMode(ControlMode mode) {
    m_controlMode = mode;
  }

  public void setManualPower(double power) {
    m_manualPower = power;
  }

  public void incrementTarget(double increment) {
    m_targetPosition =
        m_climb.getPosition().getValueAsDouble() + (increment * JOYSTICK_TO_MOTOR_ROTATIONS);
  }

  public double getClimbAngle() {
    return (m_climb.getPosition().getValueAsDouble() / ClimbInfo.MOTOR_ROT_PER_CLIMB_ROT) * 360.0;
  }

  @Override
  public void log() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();
    m_logger.log("CurrentPositionRotations", climbPosition);
    m_logger.log("SeesCoral", bannerSensorSeesCoral());
    m_logger.log("CurrentMode", m_controlMode.toString());
    m_climb.log();
    m_logger.log("CurrentManualPower", m_manualPower);
    m_logger.log("Target Position", m_targetPosition);
    m_logger.log("Angle", getClimbAngle());
  }

  @Override
  public void syncSensors() {
    if (m_climb.getPosition().getValueAsDouble() >= 225.0) {
      m_climbStopSignaler.enable();
    } else if (m_climb.getPosition().getValueAsDouble() >= 90.0) {
      m_climbHorizontalSignaler.enable();
    }
  }

  @Override
  public void update() {
    switch (m_controlMode) {
      case OffState:
        m_climb.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0, 0);
        break;
      case ClimbLow:
        // m_climb.setControl(GreyTalonFX.ControlMode.MotionMagicVoltage, 82.6, 0);
        m_climb.setControl(GreyTalonFX.ControlMode.MotionMagicVoltage, m_targetPosition, 0);
        break;
      case ClimbStow:
        m_climb.setControl(GreyTalonFX.ControlMode.MotionMagicVoltage, 0, 1);
        break;
      case JoystickMode:
        m_climb.setControl(GreyTalonFX.ControlMode.DutyCycleOut, m_manualPower);
        break;
    }
  }

  @Override
  public void reset() {}
}
