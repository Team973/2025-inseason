package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_armMotor;
  private ControlStatus m_mode = ControlStatus.Stow;
  private double m_armTargetPostionDeg;
  private double m_manualArmPower;
  private boolean m_lastHallSensorMode;
  private final DigitalInput m_hallSesnsor = new DigitalInput(4);

  public static final double HIGH_POSTION_DEG = 0;
  public static final double MEDIUM_POSTION_DEG = -30;
  public static final double LOW_POSTION_DEG = -60;
  private static final double ARM_ROTATIONS_PER_MOTOR_ROTATIONS = (10.0 / 64.0) * (24.0 / 80.0);
  private static final double CENTER_GRAVITY_OFFSET_DEG = 3;
  private static final double FEED_FORWARD_MAX_VOLT = 0.5;

  public static enum ControlStatus {
    Manual,
    TargetPostion,
    Stow,
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
    armMotorConfig.Slot0.kP = 8.0;
    armMotorConfig.Slot0.kI = 0.0;
    armMotorConfig.Slot0.kD = 0.0;
    armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 64.0;
    armMotorConfig.MotionMagic.MotionMagicAcceleration = 80.0;
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

  private void armZeroing() {
    if (m_hallSesnsor.get() == true) {
      m_armMotor.setPosition(0.0);
    }
  }

  public void setArmMotorManualOutput(double joystick) {
    m_mode = ControlStatus.Manual;
    m_manualArmPower = joystick * 0.1;
  }

  public void setArmTargetDeg(double setPostionDeg) {
    m_armTargetPostionDeg = setPostionDeg;
    m_mode = ControlStatus.TargetPostion;
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

  @Override
  public void update() {
    if (m_lastHallSensorMode = !m_hallSesnsor.get()) {
      armZeroing();
    }
    switch (m_mode) {
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
      case Stow:
        m_armMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
    }
    m_lastHallSensorMode = m_hallSesnsor.get();
  }

  public void setStow() {
    m_mode = ControlStatus.Stow;
  }

  @Override
  public void log() {
    m_logger.log(
        "armDegPostion", motorRotationsToArmDeg(m_armMotor.getPosition().getValueAsDouble()));
    m_armMotor.log();
    m_logger.log("armTargetPostionDeg", m_armTargetPostionDeg);
    m_logger.log("armMode", m_mode.toString());
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
