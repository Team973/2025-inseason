package com.team973.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Conveyor implements Subsystem {
  private final Logger m_logger;

  public final GreyTalonFX m_conveyor;

  private ControlMode m_controlMode = ControlMode.ConveyorOff;

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  public Conveyor(Logger logger) {
    m_logger = logger;
    m_conveyor = new GreyTalonFX(36, "Canivore", m_logger.subLogger("conveyor motor"));
    m_conveyor.setConfig(defaultMotorConfig());
  }

  private static TalonFXConfiguration defaultMotorConfig() {
    TalonFXConfiguration defaultMotorConfig = new TalonFXConfiguration();
    defaultMotorConfig.Slot0.kS = 0.0;
    defaultMotorConfig.Slot0.kV = 0.05;
    defaultMotorConfig.Slot0.kA = 0.0;
    defaultMotorConfig.Slot0.kP = 0.0;
    defaultMotorConfig.Slot0.kI = 0.0;
    defaultMotorConfig.Slot0.kD = 0.0;
    defaultMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 32;
    defaultMotorConfig.MotionMagic.MotionMagicAcceleration = 32;
    defaultMotorConfig.MotionMagic.MotionMagicJerk = 160;
    // slot 1 is for velocity
    defaultMotorConfig.Slot1.kS = 0.0;
    defaultMotorConfig.Slot1.kV = 0.05;
    defaultMotorConfig.Slot1.kA = 0.0;
    defaultMotorConfig.Slot1.kP = 0.0;
    defaultMotorConfig.Slot1.kI = 0.0;
    defaultMotorConfig.Slot1.kD = 0.0;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
    defaultMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    defaultMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    defaultMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    defaultMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    defaultMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    return defaultMotorConfig;
  }

  public enum ControlMode {
    ConveyorForward,
    ConveyorBackward,
    ConveyorOff,
  }

  public void setControlMode(ControlMode mode) {
    m_controlMode = mode;
  }

  @Override
  public void log() {
    double conveyorPosition = m_conveyor.getPosition().getValueAsDouble();
    m_logger.log("CurrentPositionRotations", conveyorPosition);
    m_logger.log("CurrentMode", m_controlMode.toString());
    m_conveyor.log();
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    switch (m_controlMode) {
      case ConveyorForward:
        m_conveyor.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0.1, 0);
        break;
      case ConveyorBackward:
        m_conveyor.setControl(GreyTalonFX.ControlMode.DutyCycleOut, -0.1, 0);
        break;
      case ConveyorOff:
        m_conveyor.setControl(GreyTalonFX.ControlMode.DutyCycleOut, 0, 0);
        break;
    }
  }

  @Override
  public void reset() {}
}
