package com.team973.frc2025.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.lib.devices.GreyCANCoder;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.SwerveModuleConfig;
import com.team973.lib.util.mechanisms.GearedMechanism;
import com.team973.lib.util.mechanisms.LinearMechanism;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.ArrayList;
import java.util.List;

public class SwerveModule {
  public final int moduleNumber;
  private final Rotation2d m_angleOffset;

  private final GreyTalonFX m_angleMotor;
  private final GreyTalonFX m_driveMotor;
  private final GreyCANCoder m_angleEncoder;

  private final Logger m_logger;

  private final LinearMechanism m_driveMechanism =
      new LinearMechanism(DriveInfo.DRIVE_GEAR_RATIO, DriveInfo.WHEEL_DIAMETER_METERS);
  private final GearedMechanism m_angleMechanism = new GearedMechanism(DriveInfo.ANGLE_GEAR_RATIO);

  private SwerveModuleState m_lastState;

  private final TalonFXConfiguration m_driveMotorConfig;

  private final StatusSignal<Angle> m_driveMotorPositionStatusSignal;
  private final StatusSignal<AngularVelocity> m_driveMotorVelocityStatusSignal;
  private final StatusSignal<Angle> m_angleMotorPositionStatusSignal;
  private final StatusSignal<AngularVelocity> m_angleMotorVelocityStatusSignal;

  private final ArrayList<StatusSignal<?>> m_allStatusSignals = new ArrayList<>();

  public SwerveModule(int moduleNumber, SwerveModuleConfig moduleConfig, Logger logger) {
    this.moduleNumber = moduleNumber;
    m_logger = logger;
    m_angleOffset = Rotation2d.fromDegrees(moduleConfig.angleOffset);

    /* Angle Encoder Config */
    m_angleEncoder =
        new GreyCANCoder(
            moduleConfig.cancoderID, RobotInfo.CANIVORE_CANBUS, logger.subLogger("Angle Encoder"));
    configAngleEncoder();

    /* Angle Motor Config */
    m_angleMotor =
        new GreyTalonFX(
            moduleConfig.angleMotorID, RobotInfo.CANIVORE_CANBUS, logger.subLogger("Angle Motor"));
    configAngleMotor();

    /* Drive Motor Config */
    m_driveMotor =
        new GreyTalonFX(
            moduleConfig.driveMotorID, RobotInfo.CANIVORE_CANBUS, logger.subLogger("Drive Motor"));
    m_driveMotorConfig = m_driveMotor.getCurrentConfig();
    configDriveMotor();

    BaseStatusSignal.waitForAll(0.5, m_angleEncoder.getAbsolutePosition());
    resetToAbsolute();

    m_driveMotorPositionStatusSignal = m_driveMotor.getPosition();
    m_driveMotorVelocityStatusSignal = m_driveMotor.getVelocity();

    m_angleMotorPositionStatusSignal = m_angleMotor.getPosition();
    m_angleMotorVelocityStatusSignal = m_angleMotor.getVelocity();

    m_driveMotorPositionStatusSignal.setUpdateFrequency(200);
    m_driveMotorVelocityStatusSignal.setUpdateFrequency(200);
    m_angleMotorPositionStatusSignal.setUpdateFrequency(200);
    m_angleMotorVelocityStatusSignal.setUpdateFrequency(200);

    m_allStatusSignals.add(m_driveMotorPositionStatusSignal);
    m_allStatusSignals.add(m_driveMotorVelocityStatusSignal);
    m_allStatusSignals.add(m_angleMotorPositionStatusSignal);
    m_allStatusSignals.add(m_angleMotorVelocityStatusSignal);

    m_lastState = getState();
  }

  private void configAngleEncoder() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    m_angleEncoder.getConfigurator().apply(encoderConfig);
  }

  private void configAngleMotor() {
    var motorConfig = m_angleMotor.getCurrentConfig();

    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfig.Slot0.kP = DriveInfo.ANGLE_KP;
    motorConfig.Slot0.kI = DriveInfo.ANGLE_KI;
    motorConfig.Slot0.kD = DriveInfo.ANGLE_KD;
    motorConfig.Slot0.kS = DriveInfo.ANGLE_KF;

    motorConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    m_angleMotor.setConfig(motorConfig);

    resetToAbsolute();
  }

  private void configDriveMotor() {
    m_driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_driveMotorConfig.Slot0.kP = DriveInfo.DRIVE_KP;
    m_driveMotorConfig.Slot0.kI = DriveInfo.DRIVE_KI;
    m_driveMotorConfig.Slot0.kD = DriveInfo.DRIVE_KD;
    m_driveMotorConfig.Slot0.kV = DriveInfo.DRIVE_KF;

    m_driveMotorConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    m_driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    m_driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    m_driveMotor.setConfig(m_driveMotorConfig);
    m_driveMotor.setPosition(0.0);
  }

  public Rotation2d getCanCoderRotation2d() {
    return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    m_angleMotor.setRotorPositionRotation2d(
        m_angleMechanism.getRotorRotationFromOutputRotation(
            getCanCoderRotation2d().minus(m_angleOffset)));
  }

  public List<StatusSignal<?>> allStatusSignals() {
    return m_allStatusSignals;
  }

  public SwerveModuleState getState() {
    double velocityInMPS =
        m_driveMechanism.getOutputDistanceFromRotorRotation(
            Rotation2d.fromRotations(m_driveMotorVelocityStatusSignal.getValueAsDouble()));

    return new SwerveModuleState(velocityInMPS, getAngleMotorRotation2d());
  }

  /*
   * Gets the angle of the module as a Rotation2d.
   * NOTE: This must be called after the angle motor's status signals are refreshed.
   */
  public Rotation2d getAngleMotorRotation2d() {
    double compensatedRotations =
        BaseStatusSignal.getLatencyCompensatedValue(
                m_angleMotorPositionStatusSignal, m_angleMotorVelocityStatusSignal)
            .magnitude();
    return m_angleMechanism.getOutputRotationFromRotorRotation(
        Rotation2d.fromRotations(compensatedRotations));
  }

  /*
   * Gets the linear distance driven of the module in meters.
   * NOTE: This must be called after the drive motor's status signals are refreshed.
   */
  public double getDriveMotorMeters() {
    double compensatedRotations =
        BaseStatusSignal.getLatencyCompensatedValue(
                m_driveMotorPositionStatusSignal, m_driveMotorVelocityStatusSignal)
            .magnitude();
    return m_driveMechanism.getOutputDistanceFromRotorRotation(
        Rotation2d.fromRotations(compensatedRotations));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMotorMeters(), getAngleMotorRotation2d());
  }

  public double getDriveStatorCurrent() {
    return m_driveMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getDriveSupplyCurrent() {
    return m_driveMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getTurnStatorCurrent() {
    return m_angleMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getTurnSupplyCurrent() {
    return m_angleMotor.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The desired state of the module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    setDesiredState(desiredState, false);
  }

  /**
   * Sets the desired state of the module.
   *
   * @param desiredState The desired state of the module.
   * @param force If true, the module will be set to the desired state regardless of the current
   *     state. Disables optimizations such as anti-jitter.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean ignoreJitter) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // CTRE is not
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);

    Rotation2d desiredFalconVelocityInRPS =
        m_driveMechanism.getRotorRotationFromOutputDistance(desiredState.speedMetersPerSecond);

    if (desiredState.speedMetersPerSecond != m_lastState.speedMetersPerSecond) {
      m_driveMotor.setControl(
          ControlMode.VelocityVoltage, desiredFalconVelocityInRPS.getRotations());
    }

    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    if (!ignoreJitter) {
      desiredState.angle =
          (Math.abs(desiredState.speedMetersPerSecond)
                  <= (DriveInfo.MAX_VELOCITY_METERS_PER_SECOND * 0.01))
              ? m_lastState.angle
              : desiredState.angle;
    }

    // Prevent module rotation if angle is the same as the previous angle.
    if (desiredState.angle != m_lastState.angle) {
      m_angleMotor.setControl(
          ControlMode.PositionVoltage,
          m_angleMechanism.getRotorRotationFromOutputRotation(desiredState.angle).getRotations());
    }
    m_lastState = desiredState;
  }

  public void driveBrake() {
    m_driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_driveMotor.setConfig(m_driveMotorConfig);
  }

  public void driveNeutral() {
    m_driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_driveMotor.setConfig(m_driveMotorConfig);
  }

  public void log() {
    m_driveMotor.log();
    m_angleMotor.log();
    m_angleEncoder.log();

    m_logger.log("Integrated", getState().angle.getDegrees());
    m_logger.log("Position", getPosition().distanceMeters);
    m_logger.log(
        "Target Drive Velocity RPS",
        m_driveMechanism
            .getRotorRotationFromOutputDistance(m_lastState.speedMetersPerSecond)
            .getRotations());
    m_logger.log(
        "Target Angle Rot",
        m_angleMechanism.getRotorRotationFromOutputRotation(m_lastState.angle).getRotations());
  }
}
