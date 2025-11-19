package com.team973.frc2025.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXMotorWithCanCoderSim extends TalonFXMotorSim {
  private final CANcoderSimState m_simCANCoderSimState;

  public TalonFXMotorWithCanCoderSim(TalonFX motor, CANcoder encoder) {
    super(motor);
    m_simCANCoderSimState = encoder.getSimState();
  }

  @Override
  public Voltage updateControlSignal(
      Angle mechanismAngle,
      AngularVelocity mechanismVelocity,
      Angle encoderAngle,
      AngularVelocity encoderVelocity) {
    m_simCANCoderSimState.setRawPosition(mechanismAngle);
    m_simCANCoderSimState.setVelocity(mechanismVelocity);
    return super.updateControlSignal(
        mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
  }
}
