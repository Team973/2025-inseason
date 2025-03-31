package com.team973.frc2025.subsystems;

<<<<<<< HEAD
=======
import com.ctre.phoenix6.BaseStatusSignal;
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
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

<<<<<<< HEAD
  private static final double LEVEL_FOUR_POSITION_DEG = -195.0;
=======
  private static final double LEVEL_FOUR_POSITION_DEG = -185.0;
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
  private static final double LEVEL_THREE_POSITION_DEG = -191.0;
  private static final double LEVEL_TWO_POSITION_DEG = -54.0;
  private static final double LEVEL_ONE_POSITION_DEG = 4.0;

<<<<<<< HEAD
  public static final double WITHOUT_CORAL_STOW_POSITION_DEG = -14.5;
  public static final double WITH_CORAL_STOW_POSTION_DEG = 0.0;

  private static final double ALGAE_HIGH_POSITION_DEG = -144.0;
  private static final double ALGAE_LOW_POSITION_DEG = -34.0;
  public static final double ALGAE_STOW_POSITION_DEG = -5.0;

  private double m_levelOneOffset = 0.0;
  private double m_levelTwoOffset = 0.0;
  private double m_levelThreeOffset = 0.0;
  private double m_levelFourOffset = 0.0;

  private double m_algaeHighOffset = 0.0;
  private double m_algaeLowOffset = 0.0;

=======
  public static final double WITHOUT_CORAL_STOW_POSITION_DEG = -16.0;
  public static final double WITH_CORAL_STOW_POSTION_DEG = 0.0;

  private static final double NET_POSITION_DEG = -105.0; // -20.0;
  private static final double ALGAE_HIGH_POSITION_DEG = -149.0;
  private static final double ALGAE_LOW_POSITION_DEG = -34.0;
  private static final double ALGAE_FLOOR_POSITION_DEG = -88.0;
  public static final double ALGAE_STOW_POSITION_DEG = -5.0;

>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
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
<<<<<<< HEAD
    m_wristMotor.setPosition(0.0);
=======

    BaseStatusSignal.waitForAll(0.5, m_wristEncoder.getAbsolutePosition());

    m_wristMotor.setPosition(wristDegToMotorRotations(getCanCoderPostion()));
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
  }

  public static enum ControlStatus {
    Manual,
    TargetPostion,
<<<<<<< HEAD
    Zero,
=======
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
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
<<<<<<< HEAD
        return LEVEL_ONE_POSITION_DEG + m_levelOneOffset;
      case L_2:
        return LEVEL_TWO_POSITION_DEG + m_levelTwoOffset;
      case L_3:
        return LEVEL_THREE_POSITION_DEG + m_levelThreeOffset;
      case L_4:
        return LEVEL_FOUR_POSITION_DEG + m_levelFourOffset;
      case AlgaeHigh:
        return ALGAE_HIGH_POSITION_DEG + m_algaeHighOffset;
      case AlgaeLow:
        return ALGAE_LOW_POSITION_DEG + m_algaeLowOffset;
=======
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
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
      case Horizontal:
        return HORIZONTAL_POSITION_DEG;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

<<<<<<< HEAD
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

=======
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
  public double getTargetPosition() {
    return m_wristTargetPostionDeg;
  }

  public void setMotorManualOutput(double joystick) {
    m_controlStatus = ControlStatus.Manual;
    m_manualWristPower = joystick;
  }

  private double getCanCoderPostion() {
<<<<<<< HEAD
    return (m_wristEncoder.getAbsolutePosition().getValueAsDouble()) * 360.0
        - WristInfo.ENCODER_OFFSET_DEG;
=======
    return (m_wristEncoder.getAbsolutePosition().getValueAsDouble()
            - WristInfo.ENCODER_OFFSET_ROTATIONS)
        * 360.0;
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
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
<<<<<<< HEAD
      case Zero:
        m_wristMotor.setControl(ControlMode.DutyCycleOut, -0.1);
        break;
=======
>>>>>>> 613cbed40ad20f64784a9a51425cdbad90ec77b5
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
