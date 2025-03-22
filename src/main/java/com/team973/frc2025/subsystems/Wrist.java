package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.WristInfo;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.devices.GreyCANCoder;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Wrist implements Subsystem {
  private final Logger m_logger;

  private final GreyTalonFX m_wristMotor;
  private final GreyCANCoder m_encoder;

  private ControlStatus m_controlStatus = ControlStatus.Manual;

  private double m_levelOneOffset = 0.0;
  private double m_levelTwoOffset = 0.0;
  private double m_levelThreeOffset = 0.0;
  private double m_levelFourOffset = 0.0;

  private double m_algaeHighOffset = 0.0;
  private double m_algaeLowOffset = 0.0;

  private double m_manualWristPower = 0.0;
  private double m_wristTargetPostionDeg = 0.0;

  public Wrist(Logger logger) {
    m_logger = logger;

    m_wristMotor =
        new GreyTalonFX(
            WristInfo.MOTOR_ID, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("WristMotor"));
    m_encoder =
        new GreyCANCoder(
            WristInfo.ENCODER_ID, RobotInfo.CANIVORE_CANBUS, logger.subLogger("Encoder"));

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.Slot0.kS = WristInfo.WRIST_KS;
    wristMotorConfig.Slot0.kV = WristInfo.WRIST_KV;
    wristMotorConfig.Slot0.kA = WristInfo.WRIST_KA;
    wristMotorConfig.Slot0.kP = WristInfo.WRIST_KP;
    wristMotorConfig.Slot0.kI = WristInfo.WRIST_KI;
    wristMotorConfig.Slot0.kD = WristInfo.WRIST_KD;
    wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        WristInfo.WRIST_MOTION_MAGIC_CRUISE_VELOCITY;
    wristMotorConfig.MotionMagic.MotionMagicAcceleration =
        WristInfo.WRIST_MOTION_MAGIC_ACCELERATION;
    wristMotorConfig.MotionMagic.MotionMagicJerk = WristInfo.WRIST_MOTION_MAGIC_JERK;
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = WristInfo.WRIST_SATOR_CURRENT_LIMIT;
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable =
        WristInfo.WRIST_SATOR_CURRENT_LIMIT_ENABLE;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimit = WristInfo.WRIST_SUPPLY_CURRENT_LIMIT;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        WristInfo.WRIST_SUPPLY_CURRENT_LIMIT_ENABLE;
    wristMotorConfig.Voltage.PeakForwardVoltage = WristInfo.WRIST_PEAK_FORDWARD_VOLTAGE;
    wristMotorConfig.Voltage.PeakReverseVoltage = WristInfo.WRIST_PEAK_REVERSE_VOLTAGE;
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_wristMotor.setConfig(wristMotorConfig);
  }

  public static enum ControlStatus {
    Manual,
    TargetPostion,
    Zero,
    Off,
  }

  private double wristDegToMotorRotations(double wristPostionDeg) {
    return wristPostionDeg / 360.0 / WristInfo.WRIST_ROTATIONS_PER_MOTOR_ROTATIONS;
  }

  private double motorRotationsToWristDeg(double motorPostion) {
    return motorPostion * WristInfo.WRIST_ROTATIONS_PER_MOTOR_ROTATIONS * 360.0;
  }

  public double getWristPostionDeg() {
    return motorRotationsToWristDeg(m_wristMotor.getPosition().getValueAsDouble());
  }

  public void setControlStatus(ControlStatus targetpostion) {
    m_controlStatus = targetpostion;
  }

  public double getTargetDegFromLevel(ReefLevel level) {
    switch (level) {
      case L_1:
        return WristInfo.LEVEL_ONE_POSITION_DEG + m_levelOneOffset;
      case L_2:
        return WristInfo.LEVEL_TWO_POSITION_DEG + m_levelTwoOffset;
      case L_3:
        return WristInfo.LEVEL_THREE_POSITION_DEG + m_levelThreeOffset;
      case L_4:
        return WristInfo.LEVEL_FOUR_POSITION_DEG + m_levelFourOffset;
      case AlgaeHigh:
        return WristInfo.ALGAE_HIGH_POSITION_DEG + m_algaeHighOffset;
      case AlgaeLow:
        return WristInfo.ALGAE_LOW_POSITION_DEG + m_algaeLowOffset;
      case Horizontal:
        return WristInfo.HORIZONTAL_POSITION_DEG;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  public void incrementOffset(double increment, ReefLevel level) {
    switch (level) {
      case L_1:
        m_levelOneOffset += increment;
        break;
      case L_2:
        m_levelTwoOffset += increment;
        break;
      case L_3:
        m_levelThreeOffset += increment;
        break;
      case L_4:
        m_levelFourOffset += increment;
        break;
      case AlgaeHigh:
        m_algaeHighOffset += increment;
        break;
      case AlgaeLow:
        m_algaeLowOffset += increment;
        break;
      case Horizontal:
        break;
    }
  }

  public double getTargetPosition() {
    return m_wristTargetPostionDeg;
  }

  public void setMotorManualOutput(double joystick) {
    m_controlStatus = ControlStatus.Manual;
    m_manualWristPower = joystick;
  }

  public void setTargetDeg(double setPostionDeg) {
    m_wristTargetPostionDeg = setPostionDeg;
    m_controlStatus = ControlStatus.TargetPostion;
  }

  public boolean motorAtTargetRotation() {
    return (Math.abs(
            wristDegToMotorRotations(m_wristTargetPostionDeg)
                - m_wristMotor.getPosition().getValueAsDouble())
        < 0.1);
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    switch (m_controlStatus) {
      case Manual:
        m_wristMotor.setControl(ControlMode.DutyCycleOut, m_manualWristPower, 0);
        break;
      case TargetPostion:
        m_wristMotor.setControl(
            ControlMode.MotionMagicVoltage, wristDegToMotorRotations(m_wristTargetPostionDeg), 0);
        break;
      case Zero:
        m_wristMotor.setControl(ControlMode.DutyCycleOut, -0.1);
        break;
      case Off:
        m_wristMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  @Override
  public void log() {
    m_wristMotor.log();
    m_encoder.log();

    m_logger.log("wristDegPostion", getWristPostionDeg());
    m_logger.log("wristTargetPostionDeg", m_wristTargetPostionDeg);
    m_logger.log("wristMode", m_controlStatus.toString());
    m_logger.log(
        "motorwristErrorDeg",
        motorRotationsToWristDeg(m_wristMotor.getClosedLoopError().getValueAsDouble()));
    m_logger.log("manualPower", m_manualWristPower);
  }

  @Override
  public void reset() {}
}
