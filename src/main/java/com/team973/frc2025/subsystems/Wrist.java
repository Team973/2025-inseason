package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.devices.GreyCANCoder;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Wrist implements Subsystem {
  private final RobotInfo.WristInfo m_wristInfo;
  private final Logger m_logger;

  private final GreyTalonFX m_wristMotor;
  private final GreyCANCoder m_wristEncoder;

  private ControlStatus m_controlStatus = ControlStatus.Off;

  private double m_wristTargetPostionDeg = 0.0;

  public Wrist(Logger logger, RobotInfo robotInfo) {
    m_logger = logger;
    m_wristInfo = RobotConfig.get().WRIST_INFO;

    m_wristMotor =
        new GreyTalonFX(
            m_wristInfo.MOTOR_CAN_ID, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("WristMotor"));
    m_wristEncoder =
        new GreyCANCoder(
            m_wristInfo.ENCODER_CAN_ID, RobotInfo.CANIVORE_CANBUS, logger.subLogger("Encoder"));

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.Slot0.kS = m_wristInfo.WRIST_KS;
    wristMotorConfig.Slot0.kV = m_wristInfo.WRIST_KV;
    wristMotorConfig.Slot0.kA = m_wristInfo.WRIST_KA;
    wristMotorConfig.Slot0.kP = m_wristInfo.WRIST_KP;
    wristMotorConfig.Slot0.kI = m_wristInfo.WRIST_KI;
    wristMotorConfig.Slot0.kD = m_wristInfo.WRIST_KD;

    // Motion Magic settings
    wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        m_wristInfo.WRIST_MOTION_MAGIC_CRUISE_VELOCITY;
    wristMotorConfig.MotionMagic.MotionMagicAcceleration =
        m_wristInfo.WRIST_MOTION_MAGIC_ACCELERATION;
    wristMotorConfig.MotionMagic.MotionMagicJerk = m_wristInfo.WRIST_MOTION_MAGIC_JERK;

    // Current limits
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = m_wristInfo.WRIST_SATOR_CURRENT_LIMIT;
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable =
        m_wristInfo.WRIST_SATOR_CURRENT_LIMIT_ENABLE;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimit = m_wristInfo.WRIST_SUPPLY_CURRENT_LIMIT;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        m_wristInfo.WRIST_SUPPLY_CURRENT_LIMIT_ENABLE;

    // Voltage limits
    wristMotorConfig.Voltage.PeakForwardVoltage = m_wristInfo.WRIST_PEAK_FORDWARD_VOLTAGE;
    wristMotorConfig.Voltage.PeakReverseVoltage = m_wristInfo.WRIST_PEAK_REVERSE_VOLTAGE;
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_wristMotor.setConfig(wristMotorConfig);

    BaseStatusSignal.waitForAll(0.5, m_wristEncoder.getAbsolutePosition());

    m_wristMotor.setPosition(wristDegToMotorRotations(getCanCoderPostion()));
  }

  public static enum ControlStatus {
    TargetPostion,
    Off,
  }

  private double wristDegToMotorRotations(double wristPostionDeg) {
    return wristPostionDeg / 360.0 / m_wristInfo.WRIST_ROTATIONS_PER_MOTOR_ROTATIONS;
  }

  private double motorRotationsToWristDeg(double motorPostion) {
    return motorPostion * m_wristInfo.WRIST_ROTATIONS_PER_MOTOR_ROTATIONS * 360.0;
  }

  public double getWristPostionDegFromMotorRot(double motorRot) {
    return motorRotationsToWristDeg(motorRot);
  }

  public void setControlStatus(ControlStatus targetpostion) {
    m_controlStatus = targetpostion;
  }

  public double getTargetDegFromLevel(ReefLevel level) {
    switch (level) {
      case L_1:
        return m_wristInfo.LEVEL_ONE_POSITION_DEG;
      case L_2:
        return m_wristInfo.LEVEL_TWO_POSITION_DEG;
      case L_3:
        return m_wristInfo.LEVEL_THREE_POSITION_DEG;
      case L_4:
        return m_wristInfo.LEVEL_FOUR_POSITION_DEG;
      case AlgaeHigh:
        return m_wristInfo.ALGAE_HIGH_POSITION_DEG;
      case AlgaeLow:
        return m_wristInfo.ALGAE_LOW_POSITION_DEG;
      case AlgaeFloor:
        return m_wristInfo.ALGAE_FLOOR_POSITION_DEG;
      case Net:
        return m_wristInfo.NET_POSITION_DEG;
      case Processor:
        return m_wristInfo.ALGAE_STOW_POSITION_DEG;
      case Horizontal:
        return m_wristInfo.HORIZONTAL_POSITION_DEG;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  public double getTargetPosition() {
    return m_wristTargetPostionDeg;
  }

  private double getCanCoderPostion() {
    return (m_wristEncoder.getAbsolutePosition().getValueAsDouble()
            - m_wristInfo.ENCODER_OFFSET_ROTATIONS)
        * 360.0;
  }

  public void setTargetDeg(double setPostionDeg) {
    m_wristTargetPostionDeg = setPostionDeg;
    m_controlStatus = ControlStatus.TargetPostion;
  }

  private boolean motorAtTargetRotation(double motorRot) {
    return (Math.abs(wristDegToMotorRotations(m_wristTargetPostionDeg) - motorRot) < 0.1);
  }

  public boolean motorAtTargetRotation() {
    return motorAtTargetRotation(m_wristMotor.getPosition().getValueAsDouble());
  }

  public boolean isHorizontal() {
    return Math.abs(
            getWristPostionDegFromMotorRot(m_wristMotor.getPosition().getValueAsDouble()) + 90.0)
        < 0.1;
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    switch (m_controlStatus) {
      case TargetPostion:
        m_wristMotor.setControl(
            ControlMode.MotionMagicVoltage, wristDegToMotorRotations(m_wristTargetPostionDeg), 0);
        break;
      case Off:
        m_wristMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  @Override
  public void log() {
    double motorRot = m_wristMotor.getPosition().getValueAsDouble();

    m_wristMotor.log();
    m_wristEncoder.log();

    m_logger.log("Motor At Target Rot", motorAtTargetRotation(motorRot));
    m_logger.log("wristDegPostion", getWristPostionDegFromMotorRot(motorRot));
    m_logger.log("wristTargetPostionDeg", m_wristTargetPostionDeg);
    m_logger.log("wristMode", m_controlStatus.toString());
    m_logger.log("getCanCoderPostion", getCanCoderPostion());
    m_logger.log("Is Horizontal", isHorizontal());
    m_logger.log("Level 3 taget pos", m_wristInfo.LEVEL_THREE_POSITION_DEG);
  }

  @Override
  public void reset() {}
}
