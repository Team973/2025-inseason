package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climb implements Subsystem {

  private final DigitalInput m_bannerSensor = new DigitalInput(2);

  private boolean bannerSensorSeesCoral() {
    return m_bannerSensor.get();
  }

  private final Logger m_logger;

  public final GreyTalonFX m_climb;

  private double m_manualPower = 0;

  private ControlMode m_controlMode = ControlMode.OffState;

  public Climb(Logger logger) {
    m_logger = logger;
    m_climb = new GreyTalonFX(35, "Canivore", m_logger.subLogger("motor"));
    m_climb.setConfig(defaultMotorConfig());
  }

  private static TalonFXConfiguration defaultMotorConfig() {
    TalonFXConfiguration defaultMotorConfig = new TalonFXConfiguration();
    defaultMotorConfig.Slot0.kS = 0.0;
    defaultMotorConfig.Slot0.kV = 0.0;
    defaultMotorConfig.Slot0.kA = 0.0;
    defaultMotorConfig.Slot0.kP = 6.4;
    defaultMotorConfig.Slot0.kI = 0.0;
    defaultMotorConfig.Slot0.kD = 0.04;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 32;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration = 32;
    defaultMotorConfig.MotionMagic.MotionMagicJerk = 160;
    // slot 1 is for velocity
    defaultMotorConfig.Slot1.kS = 0.0;
    defaultMotorConfig.Slot1.kV = 0;
    defaultMotorConfig.Slot1.kA = 0.0;
    defaultMotorConfig.Slot1.kP = 6.4;
    defaultMotorConfig.Slot1.kI = 0.0;
    defaultMotorConfig.Slot1.kD = 0.04;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    defaultMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    return defaultMotorConfig;
  }

  public enum ControlMode {
    ClimbLow,
    ClimbHigh,
    OffState,
    JoystickMode,
  }

  public void setControlMode(ControlMode mode) {
    m_controlMode = mode;
  }

  public void setManualPower(double power) {
    m_manualPower = power;
  }

  @Override
  public void log() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();
    m_logger.log("CurrentPositionRotations", climbPosition);
    m_logger.log("SeesCoral", bannerSensorSeesCoral());
    m_logger.log("CurrentMode", m_controlMode.toString());
    m_climb.log();
    m_logger.log("CurrentManualPower", m_manualPower);
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    switch (m_controlMode) {
      case OffState:
        m_climb.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0, 0);
        break;
      case ClimbLow:
        m_climb.setControl(GreyTalonFX.ControlMode.MotionMagicVoltage, -10, 0);
        break;
      case ClimbHigh:
        m_climb.setControl(GreyTalonFX.ControlMode.MotionMagicVoltage, 10, 1);
        break;
      case JoystickMode:
        m_climb.setControl(GreyTalonFX.ControlMode.DutyCycleOut, m_manualPower);
        break;
    }
  }

  @Override
  public void reset() {}
}
