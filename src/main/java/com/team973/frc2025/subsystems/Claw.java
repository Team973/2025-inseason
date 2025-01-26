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

public class Claw implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_algeMotor;
  private final GreyTalonFX m_clawMotor;
  private final DigitalInput m_bannerSensorFront = new DigitalInput(1);
  private final DigitalInput m_bannerSensorBack = new DigitalInput(0);
  private final DigitalInput m_algeSensor = new DigitalInput(2);
  private ControlStatus m_mode = ControlStatus.Stop;
  private ControlStatus m_lastMode = ControlStatus.Stop;
  private double m_algeTargetPostion = 0;
  private double m_clawTargetPostion = 0;

  public Claw(Logger logger) {
    m_logger = logger;
    m_clawMotor = new GreyTalonFX(37, RobotInfo.CANIVORE_CANBUS, new Logger("claw motor"));
    m_algeMotor = new GreyTalonFX(36, RobotInfo.CANIVORE_CANBUS, new Logger("claw motor"));
    TalonFXConfiguration clawMotorConfig = defaultMotorConfig();
    clawMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_clawMotor.setConfig(clawMotorConfig);
    TalonFXConfiguration algeMotorCongfig = defaultMotorConfig();
    algeMotorCongfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_algeMotor.setConfig(algeMotorCongfig);
  }

  private static TalonFXConfiguration defaultMotorConfig() {
    TalonFXConfiguration defaultMotorConfig = new TalonFXConfiguration();
    defaultMotorConfig.Slot0.kS = 0.0;
    defaultMotorConfig.Slot0.kV = 0.15;
    defaultMotorConfig.Slot0.kA = 0.01;
    defaultMotorConfig.Slot0.kP = 4.0;
    defaultMotorConfig.Slot0.kI = 0.0;
    defaultMotorConfig.Slot0.kD = 0.0;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 8;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration = 16;
    defaultMotorConfig.MotionMagic.MotionMagicJerk = 160;
    // slot 1 is for velocity
    defaultMotorConfig.Slot1.kS = 0.0;
    defaultMotorConfig.Slot1.kV = 0.125 * 10.0 / 10.5;
    defaultMotorConfig.Slot1.kA = 0.0;
    defaultMotorConfig.Slot1.kP = 0.3;
    defaultMotorConfig.Slot1.kI = 0.0;
    defaultMotorConfig.Slot1.kD = 0.0;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    return defaultMotorConfig;
  }

  public boolean algeSensorSeesAlge() {
    return m_algeSensor.get();
  }

  public boolean frontBannerSensorSeesCoral() {
    return m_bannerSensorFront.get();
  }

  public boolean backBannerSensorSeesCoral() {
    return m_bannerSensorBack.get();
  }

  public boolean clawScoreComplete() {
    return (Math.abs(m_clawTargetPostion - m_clawMotor.getPosition().getValueAsDouble()) < 0.1);
  }

  public boolean algeScoreComplete() {
    return (Math.abs(m_algeTargetPostion - m_clawMotor.getPosition().getValueAsDouble()) < 0.1);
  }

  public static enum ControlStatus {
    AlgeIntakeHold,
    IntakeAndHold,
    Shoot,
    Stop,
    Retract,
    Score,
  }

  @Override
  public void update() {
    switch (m_mode) {
      case AlgeIntakeHold:
        if (!algeSensorSeesAlge()) {
          m_algeMotor.setControl(ControlMode.DutyCycleOut, 1);
        } else {
          m_algeMotor.setControl(ControlMode.DutyCycleOut, 0);
        }
        break;
      case IntakeAndHold:
        if (frontBannerSensorSeesCoral() && !backBannerSensorSeesCoral()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0);
        } else if (!frontBannerSensorSeesCoral() && !backBannerSensorSeesCoral()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, -0.05);
        } else if (!frontBannerSensorSeesCoral() && backBannerSensorSeesCoral()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0.05);
        } else if (frontBannerSensorSeesCoral() && backBannerSensorSeesCoral()) {
          m_clawMotor.setControl(ControlMode.DutyCycleOut, 0.05);
        }
        break;
      case Shoot:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 2, 1);
        break;
      case Stop:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, 0, 1);
        break;
      case Score:
        if (m_lastMode != m_mode) {
          m_clawTargetPostion = m_clawMotor.getPosition().getValueAsDouble() + 4.5;
          m_algeTargetPostion = m_algeMotor.getPosition().getValueAsDouble() + 4.5;
        }
        m_clawMotor.setControl(ControlMode.MotionMagicVoltage, m_algeTargetPostion, 0);
        break;
      case Retract:
        m_clawMotor.setControl(ControlMode.VelocityVoltage, -5, 1);
        break;
    }
    m_lastMode = m_mode;
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  @Override
  public void log() {

    m_logger.log("backBannerSensor", backBannerSensorSeesCoral());
    m_logger.log("frontBannerSensor", frontBannerSensorSeesCoral());
    m_logger.log("target postion alge", algeSensorSeesAlge());
    m_algeMotor.log();
    m_clawMotor.log();
    m_logger.log("target postion left", m_algeTargetPostion);
    m_logger.log("target postion right", m_clawTargetPostion);
    m_logger.log("target rotations hit", clawScoreComplete());
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
