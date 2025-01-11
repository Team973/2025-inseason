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
  private final GreyTalonFX m_motorRight;
  private final GreyTalonFX m_motorLeft;
  private final DigitalInput m_sensor = new DigitalInput(0);
  private ControlStatus m_mode = ControlStatus.Stop;

  public Claw(Logger logger) {
    m_logger = logger;
    m_motorRight = new GreyTalonFX(36, "Canivore", new Logger("shooterRight"));
    m_motorLeft = new GreyTalonFX(35, "Canivore", new Logger("shooterLeft"));

    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motorRight.setConfig(rightMotorConfig);
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motorLeft.setConfig(leftMotorConfig);
  }

  private boolean sensorSeeCoral() {
    return !m_sensor.get();
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
        if (sensorSeeCoral()) {
          m_motorRight.setControl(ControlMode.DutyCycleOut, 0);
          m_motorLeft.setControl(ControlMode.DutyCycleOut, 0);
        } else if (!sensorSeeCoral()) {
          m_motorRight.setControl(ControlMode.DutyCycleOut, 0.05);
          m_motorLeft.setControl(ControlMode.DutyCycleOut, 0.05);
        }
        break;
      case Shoot:
        m_motorRight.setControl(ControlMode.DutyCycleOut, 0.1);
        m_motorLeft.setControl(ControlMode.DutyCycleOut, 0.1);
        break;
      case Stop:
        m_motorRight.setControl(ControlMode.DutyCycleOut, 0);
        m_motorLeft.setControl(ControlMode.DutyCycleOut, 0);
        break;
      case Retract:
        m_motorRight.setControl(ControlMode.DutyCycleOut, -0.1);
        m_motorLeft.setControl(ControlMode.DutyCycleOut, -0.1);
        break;
    }
  }

  public void setControl(ControlStatus mode) {
    m_mode = mode;
  }

  @Override
  public void log() {
    m_logger.log("sensorSeeCoral", sensorSeeCoral());
    m_motorRight.log();
    m_motorLeft.log();
  }

  @Override
  public void syncSensors() {}

  @Override
  public void reset() {}
}
