package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
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
  private final GreyCANCoder m_wristEncoder;

  private ControlStatus m_controlStatus = ControlStatus.Manual;

  private static final double HORIZONTAL_POSITION_DEG = 0.0;

  private static final double LEVEL_FOUR_POSITION_DEG = -187.0;
  private static final double LEVEL_THREE_POSITION_DEG = -189.0;
  private static final double LEVEL_TWO_POSITION_DEG = -54.0;
  private static final double LEVEL_ONE_POSITION_DEG = 4.0;

  public static final double WITHOUT_CORAL_STOW_POSITION_DEG = -17.0;
  public static final double WITH_CORAL_STOW_POSTION_DEG = 0.0;

  private static final double NET_POSITION_DEG = -105.0; // -20.0;
  private static final double ALGAE_HIGH_POSITION_DEG = -149.0;
  private static final double ALGAE_LOW_POSITION_DEG = -34.0;
  private static final double ALGAE_FLOOR_POSITION_DEG = -88.0;
  public static final double ALGAE_STOW_POSITION_DEG = -5.0;

  private double m_manualWristPower = 0.0;
  private double m_wristTargetPostionDeg = 0.0;

  public Wrist(Logger logger) {
    m_logger = logger;

    m_wristMotor =
        new GreyTalonFX(
            WristInfo.MOTOR_CAN_ID, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("WristMotor"));
    m_wristEncoder =
        new GreyCANCoder(
            WristInfo.ENCODER_CAN_ID, RobotInfo.CANIVORE_CANBUS, logger.subLogger("Encoder"));

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.Slot0.kS = 0.0;
    wristMotorConfig.Slot0.kV = 0.0;
    wristMotorConfig.Slot0.kA = 0.0;
    wristMotorConfig.Slot0.kP = 10.0; // 10
    wristMotorConfig.Slot0.kI = 0.0;
    wristMotorConfig.Slot0.kD = 0.0;
    wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 51.0; // 51.0;
    wristMotorConfig.MotionMagic.MotionMagicAcceleration = 590.0; // 80.0;
    wristMotorConfig.MotionMagic.MotionMagicJerk = 5900.0;
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 20.0;
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
    wristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wristMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    wristMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_wristMotor.setConfig(wristMotorConfig);

    BaseStatusSignal.waitForAll(0.5, m_wristEncoder.getAbsolutePosition());

    m_wristMotor.setPosition(wristDegToMotorRotations(getCanCoderPostion()));
  }

  public static enum ControlStatus {
    Manual,
    TargetPostion,
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
        return LEVEL_ONE_POSITION_DEG;
      case L_2:
        return LEVEL_TWO_POSITION_DEG;
      case L_3:
        return LEVEL_THREE_POSITION_DEG;
      case L_4:
        return LEVEL_FOUR_POSITION_DEG;
      case AlgaeHigh:
        return ALGAE_HIGH_POSITION_DEG;
      case AlgaeLow:
        return ALGAE_LOW_POSITION_DEG;
      case AlgaeFloor:
        return ALGAE_FLOOR_POSITION_DEG;
      case Net:
        return NET_POSITION_DEG;
      case Processor:
        return ALGAE_STOW_POSITION_DEG;
      case Horizontal:
        return HORIZONTAL_POSITION_DEG;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  public double getTargetPosition() {
    return m_wristTargetPostionDeg;
  }

  public void setMotorManualOutput(double joystick) {
    m_controlStatus = ControlStatus.Manual;
    m_manualWristPower = joystick;
  }

  private double getCanCoderPostion() {
    return (m_wristEncoder.getAbsolutePosition().getValueAsDouble()
            - WristInfo.ENCODER_OFFSET_ROTATIONS)
        * 360.0;
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
      case Off:
        m_wristMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  @Override
  public void log() {
    m_wristMotor.log();
    m_wristEncoder.log();

    m_logger.log("wristDegPostion", getWristPostionDeg());
    m_logger.log("wristTargetPostionDeg", m_wristTargetPostionDeg);
    m_logger.log("wristMode", m_controlStatus.toString());
    m_logger.log(
        "motorwristErrorDeg",
        motorRotationsToWristDeg(m_wristMotor.getClosedLoopError().getValueAsDouble()));
    m_logger.log("manualPower", m_manualWristPower);
    m_logger.log("getCanCoderPostion", getCanCoderPostion());
  }

  @Override
  public void reset() {}
}
