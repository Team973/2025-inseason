package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Arm implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_armMotor;
  private ControlStatus m_controlStatus = ControlStatus.Off;
  private double m_armTargetPostionDeg;
  private double m_manualArmPower;

  private static final double LEVEL_FOUR_POSITION_DEG = 68.0;
  private static final double LEVEL_THREE_POSITION_DEG = 60;
  private static final double LEVEL_TWO_POSITION_DEG = -70.0;
  private static final double LEVEL_ONE_POSITION_DEG = -70.0;
  public static final double STOW_POSITION_DEG = -94.0;

  private static final double ARM_ROTATIONS_PER_MOTOR_ROTATIONS = (10.0 / 64.0) * (24.0 / 80.0);
  private static final double CENTER_GRAVITY_OFFSET_DEG = 3;
  private static final double FEED_FORWARD_MAX_VOLT = 0.6; // 0.5;

  public static enum ControlStatus {
    Manual,
    TargetPostion,
    Off,
  }

  private double armDegToMotorRotations(double armPostionDeg) {
    return armPostionDeg / 360.0 / ARM_ROTATIONS_PER_MOTOR_ROTATIONS;
  }

  private double motorRotationsToArmDeg(double motorPostion) {
    return motorPostion * ARM_ROTATIONS_PER_MOTOR_ROTATIONS * 360.0;
  }

  public Arm(Logger logger) {
    m_logger = logger;
    m_armMotor = new GreyTalonFX(30, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("armMotor"));
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.Slot0.kS = 0.0;
    armMotorConfig.Slot0.kV = 0.0;
    armMotorConfig.Slot0.kA = 0.0;
    armMotorConfig.Slot0.kP = 50.0;
    armMotorConfig.Slot0.kI = 0.0;
    armMotorConfig.Slot0.kD = 0.0;
    armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 72.0; // 64.0;
    armMotorConfig.MotionMagic.MotionMagicAcceleration = 90.0; // 80.0;
    armMotorConfig.MotionMagic.MotionMagicJerk = 160.0;
    armMotorConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    armMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    armMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    armMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    // clockwise postive when looking at it from the shaft, is on inside of the left point so that
    // the shaft is pointing left, up is positve, gear ratio is odd
    armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_armMotor.setConfig(armMotorConfig);
    m_armMotor.setPosition(armDegToMotorRotations(-90.0));
  }

  public void setMotorManualOutput(double joystick) {
    m_controlStatus = ControlStatus.Manual;
    m_manualArmPower = joystick * 0.1;
  }

  public void setTargetDeg(double setPostionDeg) {
    m_armTargetPostionDeg = setPostionDeg;
    m_controlStatus = ControlStatus.TargetPostion;
  }

  public boolean motorAtTargetRotation() {
    return (Math.abs(
            armDegToMotorRotations(m_armTargetPostionDeg)
                - m_armMotor.getPosition().getValueAsDouble())
        < 0.1);
  }

  private double getFeedForwardTargetAngle() {
    double armAngleDeg = motorRotationsToArmDeg(m_armMotor.getPosition().getValueAsDouble());
    return FEED_FORWARD_MAX_VOLT
        * Math.cos((armAngleDeg - CENTER_GRAVITY_OFFSET_DEG) * (Math.PI / 180));
  }

  public static double getTargetDegFromLevel(int level) {
    switch (level) {
      case 1:
        return LEVEL_ONE_POSITION_DEG;
      case 2:
        return LEVEL_TWO_POSITION_DEG;
      case 3:
        return LEVEL_THREE_POSITION_DEG;
      case 4:
        return LEVEL_FOUR_POSITION_DEG;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  @Override
  public void update() {
    switch (m_controlStatus) {
      case Manual:
        m_armMotor.setControl(
            ControlMode.VoltageOut, (m_manualArmPower * 12.0) + getFeedForwardTargetAngle(), 0);
        break;
      case TargetPostion:
        m_armMotor.setControl(
            ControlMode.MotionMagicVoltage,
            armDegToMotorRotations(m_armTargetPostionDeg),
            getFeedForwardTargetAngle(),
            0);
        break;
      case Off:
        m_armMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
    }
  }

  public void setControlStatus(ControlStatus status) {
    m_controlStatus = status;
  }

  @Override
  public void log() {
    m_logger.log(
        "armDegPostion", motorRotationsToArmDeg(m_armMotor.getPosition().getValueAsDouble()));
    m_armMotor.log();
    m_logger.log("armTargetPostionDeg", m_armTargetPostionDeg);
    m_logger.log("armMode", m_controlStatus.toString());
    m_logger.log(
        "motorArmErrorDeg",
        motorRotationsToArmDeg(m_armMotor.getClosedLoopError().getValueAsDouble()));
    m_logger.log("ArmFeedForwardTarget", getFeedForwardTargetAngle());
    m_logger.log("manualPower", m_manualArmPower);
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
