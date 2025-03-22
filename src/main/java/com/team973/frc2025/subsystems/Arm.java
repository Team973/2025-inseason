package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_armMotor;
  private ControlStatus m_controlStatus = ControlStatus.Off;
  private double m_armTargetPostionDeg;
  private double m_manualArmPower;
  // We want to track the transition from hall=false to hall=true; when the robot
  // first boots up, we don't really know what the previous hall state was, so we
  // assume it was true to avoid false positive.
  private boolean m_lastHallSensorMode = true;
  private final DigitalInput m_hallSesnsor = new DigitalInput(RobotInfo.ArmInfo.HALL_SENSOR_ID);

  private final SolidSignaler m_armHomedSigaler =
      new SolidSignaler(
          RobotInfo.Colors.BLUE, 250, RobotInfo.SignalerInfo.ARM_HALL_SENSOR_SIGNALER_PRIORTY);
  private double m_levelOneOffset = 0.0;
  private double m_levelTwoOffset = 0.0;
  private double m_levelThreeOffset = 0.0;
  private double m_levelFourOffset = 0.0;

  private double m_algaeHighOffset = 0.0;
  private double m_algaeLowOffset = 0.0;

  private CANdleManger m_candleManger;

  public static enum ControlStatus {
    Manual,
    TargetPostion,
    Zero,
    Off,
  }

  private double armDegToMotorRotations(double armPostionDeg) {
    return armPostionDeg / 360.0 / RobotInfo.ArmInfo.ARM_ROTATIONS_PER_MOTOR_ROTATIONS;
  }

  private double motorRotationsToArmDeg(double motorPostion) {
    return motorPostion * RobotInfo.ArmInfo.ARM_ROTATIONS_PER_MOTOR_ROTATIONS * 360.0;
  }

  public Arm(Logger logger, CANdleManger candle) {
    m_logger = logger;
    m_armMotor = new GreyTalonFX(30, RobotInfo.CANIVORE_CANBUS, m_logger.subLogger("armMotor"));
    m_candleManger = candle;
    m_candleManger.addSignaler(m_armHomedSigaler);
    TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
    armMotorConfig.Slot0.kS = RobotInfo.ArmInfo.ARM_KS;
    armMotorConfig.Slot0.kV = RobotInfo.ArmInfo.ARM_KV;
    armMotorConfig.Slot0.kA = RobotInfo.ArmInfo.ARM_KA;
    armMotorConfig.Slot0.kP = RobotInfo.ArmInfo.ARM_KP;
    armMotorConfig.Slot0.kI = RobotInfo.ArmInfo.ARM_KI;
    armMotorConfig.Slot0.kD = RobotInfo.ArmInfo.ARM_KD;
    armMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        RobotInfo.ArmInfo.ARM_MOTION_MAGIC_CRUISE_VELOCITY;
    armMotorConfig.MotionMagic.MotionMagicAcceleration =
        RobotInfo.ArmInfo.ARM_MOTION_MAGIC_ACCELERATION;
    armMotorConfig.MotionMagic.MotionMagicJerk = RobotInfo.ArmInfo.ARM_MOTION_MAGIC_JERK;
    armMotorConfig.CurrentLimits.StatorCurrentLimit = RobotInfo.ArmInfo.ARM_SATOR_CURRENT_LIMIT;
    armMotorConfig.CurrentLimits.StatorCurrentLimitEnable =
        RobotInfo.ArmInfo.ARM_SATOR_CURRENT_LIMIT_ENABLE;
    armMotorConfig.CurrentLimits.SupplyCurrentLimit = RobotInfo.ArmInfo.ARM_SUPPLY_CURRENT_LIMIT;
    armMotorConfig.CurrentLimits.SupplyCurrentLimitEnable =
        RobotInfo.ArmInfo.ARM_SUPPLY_CURRENT_LIMIT_ENABLE;
    armMotorConfig.Voltage.PeakForwardVoltage = RobotInfo.ArmInfo.ARM_PEAK_FORDWARD_VOLTAGE;
    armMotorConfig.Voltage.PeakReverseVoltage = RobotInfo.ArmInfo.ARM_PEAK_REVERSE_VOLTAGE;
    // clockwise postive when looking at it from the shaft, is on inside of the left
    // point so that
    // the shaft is pointing left, up is positve, gear ratio is odd
    armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_armMotor.setConfig(armMotorConfig);
    m_armMotor.setPosition(armDegToMotorRotations(RobotInfo.ArmInfo.ARM_HOMING_POSTION_DEG));
  }

  private boolean hallSensor() {
    return !m_hallSesnsor.get();
  }

  private void maybeHomeArm() {
    if (m_lastHallSensorMode == false && hallSensor() == true) {
      m_armMotor.setPosition(armDegToMotorRotations(RobotInfo.ArmInfo.ARM_HOMING_POSTION_DEG));
      m_armHomedSigaler.enable();
    }
    m_lastHallSensorMode = hallSensor();
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
    return RobotInfo.ArmInfo.FEED_FORWARD_MAX_VOLT
        * Math.cos((armAngleDeg - RobotInfo.ArmInfo.CENTER_GRAVITY_OFFSET_DEG) * (Math.PI / 180));
  }

  public double getTargetDegFromLevel(ReefLevel level) {
    switch (level) {
      case L_1:
        return RobotInfo.ArmInfo.LEVEL_ONE_POSITION_DEG + m_levelOneOffset;
      case L_2:
        return RobotInfo.ArmInfo.LEVEL_TWO_POSITION_DEG + m_levelTwoOffset;
      case L_3:
        return RobotInfo.ArmInfo.LEVEL_THREE_POSITION_DEG + m_levelThreeOffset;
      case L_4:
        return RobotInfo.ArmInfo.LEVEL_FOUR_POSITION_DEG + m_levelFourOffset;
      case AlgaeHigh:
        return RobotInfo.ArmInfo.ALGAE_HIGH_POSITION_DEG + m_algaeHighOffset;
      case AlgaeLow:
        return RobotInfo.ArmInfo.ALGAE_LOW_POSITION_DEG + m_algaeLowOffset;

      default:
        throw new IllegalArgumentException(String.valueOf(level));
    }
  }

  public double getTargetPosition() {
    return m_armTargetPostionDeg;
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
      case Zero:
        m_armMotor.setControl(ControlMode.DutyCycleOut, -0.1);
        break;
      case Off:
        m_armMotor.setControl(ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  public double getArmPostionDeg() {
    return motorRotationsToArmDeg(m_armMotor.getPosition().getValueAsDouble());
  }

  public void setControlStatus(ControlStatus status) {
    m_controlStatus = status;
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

  @Override
  public void log() {
    m_logger.log("armDegPostion", getArmPostionDeg());
    m_armMotor.log();
    m_logger.log("armTargetPostionDeg", m_armTargetPostionDeg);
    m_logger.log("armMode", m_controlStatus.toString());
    m_logger.log(
        "motorArmErrorDeg",
        motorRotationsToArmDeg(m_armMotor.getClosedLoopError().getValueAsDouble()));
    m_logger.log("ArmFeedForwardTarget", getFeedForwardTargetAngle());
    m_logger.log("manualPower", m_manualArmPower);
    m_logger.log("HallsensorArm", hallSensor());

    m_logger.log("Level 1 Offset", m_levelOneOffset);
    m_logger.log("Level 2 Offset", m_levelTwoOffset);
    m_logger.log("Level 3 Offset", m_levelThreeOffset);
    m_logger.log("Level 4 Offset", m_levelFourOffset);
    m_logger.log("Algae Low Offset", m_algaeLowOffset);
    m_logger.log("Algae High Offset", m_algaeHighOffset);
  }

  @Override
  public void syncSensors() {
    maybeHomeArm();
  }

  @Override
  public void reset() {}
}
