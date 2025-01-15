package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.devices.GreyTalonFX.ControlMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Claw implements Subsystem {
  private final Logger m_logger;
  private final GreyTalonFX m_clawMotor;
  private final DigitalInput m_bannerSensorFront = new DigitalInput(1);
  private final DigitalInput m_bannerSensorBack = new DigitalInput(0);
  private ControlStatus m_mode = ControlStatus.Stop;

  public Claw(Logger logger) {
    m_logger = logger;
    m_clawMotor = new GreyTalonFX(36, "Canivore", new Logger("ClawMotor"));

    TalonFXConfiguration clawMotorConfig = new TalonFXConfiguration();
    clawMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_clawMotor.setConfig(clawMotorConfig);
    
  }

  public boolean frontBannerSensorSeesCoral() {
    return m_bannerSensorFront.get();
  }

  public boolean backBannerSensorSeesCoral() {
    return m_bannerSensorBack.get();
  }

  public static enum ControlStatus {
    IntakeAndHold,
    Shoot,
    Stop,
    Retract,
  }

  @Override
  public void update() {
    switch (m_mode) {
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
        m_clawMotor.setControl(ControlMode.DutyCycleOut, 0.1);
        break;
      case Stop:
        m_clawMotor.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case Retract:
        m_clawMotor.setControl(ControlMode.DutyCycleOut, -0.1);
        break;
    }
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  @Override
  public void log() {
    m_logger.log("backBannerSensor", backBannerSensorSeesCoral());
    m_logger.log("frontBannerSensor", frontBannerSensorSeesCoral());
    m_clawMotor.log();
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
