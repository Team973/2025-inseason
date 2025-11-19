package com.team973.frc2025.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class TalonFXMotorSim implements SimulatedMotorController {
  private final TalonFXSimState m_simState;

  public TalonFXMotorSim(TalonFX motor) {
    m_simState = motor.getSimState();
  }

  @Override
  public Voltage updateControlSignal(
      Angle mechanismAngle,
      AngularVelocity mechanismVelocity,
      Angle encoderAngle,
      AngularVelocity encoderVelocity) {
    m_simState.setRawRotorPosition(encoderAngle);
    m_simState.setRotorVelocity(encoderVelocity);
    m_simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    return m_simState.getMotorVoltageMeasure();
  }
}
