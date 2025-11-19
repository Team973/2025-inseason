package com.team973.frc2025.subsystems.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team973.lib.util.SwerveModuleConfig;
import com.team973.lib.util.SwerveSimUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class SwerveModuleSim extends SwerveModule {
  private SwerveModuleSimulation m_sim;

  public SwerveModuleSim(
      int moduleNumber,
      SwerveModuleSimulation sim,
      SwerveModuleConfig moduleConfig,
      com.team973.lib.util.Logger logger) {
    super(moduleNumber, SwerveSimUtil.sanitizeModuleConfig(moduleConfig), logger);
    m_sim = sim;

    m_driveMotor.setConfig(
        m_driveMotor
            .getCurrentConfig()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
    m_angleMotor.setConfig(
        m_angleMotor
            .getCurrentConfig()
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));

    m_sim.useDriveMotorController(new TalonFXMotorSim(m_driveMotor));
    m_sim.useSteerMotorController(new TalonFXMotorWithCanCoderSim(m_angleMotor, m_angleEncoder));
  }
}
