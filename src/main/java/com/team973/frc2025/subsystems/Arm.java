package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Arm implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_armMotor;
  private ControlStatus m_mode = ControlStatus.Stow;
  private ControlStatus m_lastMode = ControlStatus.Stow;
  private double m_armTargetPostion;

  public static final double HIGH_POSTION_DEG = 30;
  public static final double MEDIUM_POSTION_DEG = 0;
  public static final double LOW_POSTION_DEG = -30;
  private static final double MOTOR_TO_ARM_GEAR_RATIO = 1;

  public static enum ControlStatus {
    targetPostion,
    Stow,
  }

  private double armToMotor(double armPostion) {
    return armPostion * MOTOR_TO_ARM_GEAR_RATIO;
  }
  private double motorToArm(double motorPostion){
    return motorPostion / MOTOR_TO_ARM_GEAR_RATIO;
  }

  public Arm(Logger logger) {
    m_logger = logger;
    m_armMotor = new GreyTalonFX(10, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("armMotor"));
    // clockwise postive when looking at it from the shaft, is on inside of the left point so that
    // the shaft is pointing left, up is positve, gear ratio is odd
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.Slot0.kS = 0.0;
    armMotorConfig.Slot0.kV = 0.15;
    armMotorConfig.Slot0.kA = 0.01;
    armMotorConfig.Slot0.kP = 4.0;
    armMotorConfig.Slot0.kI = 0.0;
    armMotorConfig.Slot0.kD = 0.0;
    armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    armMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    armMotorConfig.MotionMagic.MotionMagicJerk = 160;
    armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_armMotor.setConfig(armMotorConfig);
  }

  public void setArmTargetDeg(double setPostionDeg) {
    m_armTargetPostion = setPostionDeg;
    m_mode = ControlStatus.targetPostion;
  }

  public boolean motorAtTarget() {
    return (Math.abs(m_armTargetPostion - m_armMotor.getPosition().getValueAsDouble()) < 0.1);
  }

  @Override
  public void update() {
    switch (m_mode) {
      case targetPostion:
      if (m_lastMode != m_mode)  
      m_armMotor.setControl(ControlMode.MotionMagicVoltage, armToMotor(m_armTargetPostion), 0);
        break;
      case Stow:
        m_armMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
    }

    m_lastMode = m_mode;
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  @Override
  public void log() {
    m_logger.log("armPostion",motorToArm(m_armMotor.getPosition().getValueAsDouble()));
    m_armMotor.log();
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
