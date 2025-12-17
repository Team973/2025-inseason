package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.SignalerInfo;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.devices.GreyCANCoder;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.StateMap;
import com.team973.lib.util.Subsystem;

public class Arm extends Subsystem<Arm.State> {
  private final RobotInfo m_robotInfo;
  private final RobotInfo.ArmInfo m_armInfo;
  private final Logger m_logger;
  private final GreyTalonFX m_armMotor;
  private final GreyCANCoder m_armEncoder;

  private final StateMap<State> m_stateMap;

  private double m_armTargetPostionDeg;

  private final SolidSignaler m_armHorizontalSigaler;

  private double m_levelOneOffset = 0.0;
  private double m_levelTwoOffset = 0.0;
  private double m_levelThreeOffset = 0.0;
  private double m_levelFourOffset = 0.0;

  private double m_netOffset = 0.0;
  private double m_algaeHighOffset = 0.0;
  private double m_algaeLowOffset = 0.0;
  private double m_algaeFloorOffset = 0.0;

  public enum State {
    TargetPostion,
    Off,
  }

  public Arm(Logger logger, CANdleManger candle) {
    super(State.Off);

    m_robotInfo = RobotConfig.get();
    m_armInfo = m_robotInfo.ARM_INFO;
    m_logger = logger;
    m_armMotor = new GreyTalonFX(30, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("armMotor"));
    m_armEncoder =
        new GreyCANCoder(
            m_armInfo.ENCODER_CAN_ID, RobotInfo.CANIVORE_CANBUS, logger.subLogger("Encoder"));
    m_armHorizontalSigaler =
        new SolidSignaler(RobotInfo.Colors.BLUE, 250, SignalerInfo.ARM_HORIZONTAL_SIGNALER_PRIORTY);

    m_stateMap = new StateMap<>(State.class);

    candle.addSignaler(m_armHorizontalSigaler);

    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.Slot0.kS = m_armInfo.ARM_KS;
    armMotorConfig.Slot0.kV = m_armInfo.ARM_KV;
    armMotorConfig.Slot0.kA = m_armInfo.ARM_KA;
    armMotorConfig.Slot0.kP = m_armInfo.ARM_KP;
    armMotorConfig.Slot0.kI = m_armInfo.ARM_KI;
    armMotorConfig.Slot0.kD = m_armInfo.ARM_KD;
    armMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        m_armInfo.ARM_MOTION_MAGIC_CRUISE_VELOCITY;
    armMotorConfig.MotionMagic.MotionMagicAcceleration = m_armInfo.ARM_MOTION_MAGIC_ACCELERATION;
    armMotorConfig.MotionMagic.MotionMagicJerk = m_armInfo.ARM_MOTION_MAGIC_JERK;
    armMotorConfig.CurrentLimits.StatorCurrentLimit = m_armInfo.ARM_SATOR_CURRENT_LIMIT;
    armMotorConfig.CurrentLimits.StatorCurrentLimitEnable =
        m_armInfo.ARM_SATOR_CURRENT_LIMIT_ENABLE;
    armMotorConfig.CurrentLimits.SupplyCurrentLimit = m_armInfo.ARM_SUPPLY_CURRENT_LIMIT;
    armMotorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        m_armInfo.ARM_SUPPLY_CURRENT_LIMIT_ENABLE;
    armMotorConfig.Voltage.PeakForwardVoltage = m_armInfo.ARM_PEAK_FORDWARD_VOLTAGE;
    armMotorConfig.Voltage.PeakReverseVoltage = m_armInfo.ARM_PEAK_REVERSE_VOLTAGE;
    // clockwise postive when looking at it from the shaft, is on inside of the left
    // point so that
    // the shaft is pointing left, up is positve, gear ratio is odd
    armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_armMotor.setConfig(armMotorConfig);

    BaseStatusSignal.waitForAll(0.5, m_armEncoder.getAbsolutePosition());

    m_armMotor.setPosition(armDegToMotorRotations(getCanCoderPostionDeg()));
  }

  public StateMap<State> getStateMap() {
    return m_stateMap;
  }

  private double armDegToMotorRotations(double armPostionDeg) {
    return armPostionDeg / 360.0 / m_armInfo.ARM_ROTATIONS_PER_MOTOR_ROTATIONS;
  }

  private double motorRotationsToArmDeg(double motorPostion) {
    return motorPostion * m_armInfo.ARM_ROTATIONS_PER_MOTOR_ROTATIONS * 360.0;
  }

  private double getCanCoderPostionDeg() {
    return (m_armEncoder.getAbsolutePosition().getValueAsDouble()
            - m_armInfo.ENCODER_OFFSET_ROTATIONS)
        * 360.0;
  }

  public void setTargetDeg(double setPostionDeg) {
    m_armTargetPostionDeg = setPostionDeg;
    setState(State.TargetPostion);
  }

  public boolean motorAtTargetRotation() {
    return (Math.abs(
            armDegToMotorRotations(m_armTargetPostionDeg)
                - m_armMotor.getPosition().getValueAsDouble())
        < 0.1);
  }

  private double getFeedForwardTargetAngleFromMotorRot(double motorRot) {
    double armAngleDeg = motorRotationsToArmDeg(motorRot);
    return m_armInfo.FEED_FORWARD_MAX_VOLT
        * Math.cos((armAngleDeg - m_armInfo.CENTER_GRAVITY_OFFSET_DEG) * (Math.PI / 180));
  }

  public double getTargetDegFromLevel(ReefLevel level) {
    switch (level) {
      case L_1:
        return m_armInfo.LEVEL_ONE_POSITION_DEG + m_levelOneOffset;
      case L_2:
        return m_armInfo.LEVEL_TWO_POSITION_DEG + m_levelTwoOffset;
      case L_3:
        return m_armInfo.LEVEL_THREE_POSITION_DEG + m_levelThreeOffset;
      case L_4:
        return m_armInfo.LEVEL_FOUR_POSITION_DEG + m_levelFourOffset;
      case Net:
        return m_armInfo.NET_POSITION_DEG + m_netOffset;
      case AlgaeHigh:
        return m_armInfo.ALGAE_HIGH_POSITION_DEG + m_algaeHighOffset;
      case AlgaeLow:
        return m_armInfo.ALGAE_LOW_POSITION_DEG + m_algaeLowOffset;
      case AlgaeFloor:
        return m_armInfo.ALGAE_FLOOR_POSITION_DEG + m_algaeFloorOffset;
      case Processor:
        return m_armInfo.ALGAE_STOW_POSITION_DEG;
      case Horizontal:
        return m_armInfo.HORIZONTAL_POSITION_DEG;
      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  public double getTargetPosition() {
    return m_armTargetPostionDeg;
  }

  @Override
  public void update() {
    switch (getState()) {
      case TargetPostion:
        m_armMotor.setControl(
            ControlMode.MotionMagicVoltage,
            armDegToMotorRotations(m_armTargetPostionDeg),
            getFeedForwardTargetAngleFromMotorRot(m_armMotor.getPosition().getValueAsDouble()),
            0);
        break;
      case Off:
        m_armMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  public double getArmPostionDegFromMotorRot(double motorRot) {
    return motorRotationsToArmDeg(motorRot);
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
      case Net:
        m_netOffset += increment;
        break;
      case AlgaeHigh:
        m_algaeHighOffset += increment;
        break;
      case AlgaeLow:
        m_algaeLowOffset += increment;
        break;
      case AlgaeFloor:
        m_algaeFloorOffset += increment;
        break;
      case Processor:
        break;
      case Horizontal:
        break;
    }
  }

  @Override
  public void log() {
    double motorRot = m_armMotor.getPosition().getValueAsDouble();

    m_armMotor.log();
    m_armEncoder.log();

    m_logger.log("armDegPostion", getArmPostionDegFromMotorRot(motorRot));
    m_logger.log("armTargetPostionDeg", m_armTargetPostionDeg);
    m_logger.log("armMode", getState().toString());
    m_logger.log("ArmFeedForwardTarget", getFeedForwardTargetAngleFromMotorRot(motorRot));

    m_logger.log("getCanCoderPostion", getCanCoderPostionDeg());

    m_logger.log("Level 1 Offset", m_levelOneOffset);
    m_logger.log("Level 2 Offset", m_levelTwoOffset);
    m_logger.log("Level 3 Offset", m_levelThreeOffset);
    m_logger.log("Level 4 Offset", m_levelFourOffset);
    m_logger.log("Net Offset", m_netOffset);
    m_logger.log("Algae Low Offset", m_algaeLowOffset);
    m_logger.log("Algae High Offset", m_algaeHighOffset);
    m_logger.log("Algae Floor Offset", m_algaeFloorOffset);

    m_logger.log("Is Horizontal", isHorizontal());
  }

  public boolean isHorizontal() {
    return Math.abs(getArmPostionDegFromMotorRot(m_armMotor.getPosition().getValueAsDouble()))
        < 0.1;
  }

  @Override
  public void syncSensors() {
    if (isHorizontal()) {
      m_armHorizontalSigaler.enable();
    } else {
      m_armHorizontalSigaler.disable();
    }
  }

  @Override
  public void reset() {}
}
