package com.team973.lib.devices;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.team973.lib.util.Logger;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GreyPigeonSim extends GreyPigeon {
  private final GyroSimulation m_gyroSim;
  private final Pigeon2SimState m_simState;

  public GreyPigeonSim(Logger logger, GyroSimulation gyroSim) {
    super(logger);

    m_gyroSim = gyroSim;
    m_simState = m_pigeon.getSimState();
  }

  @Override
  public void simulationUpdate() {
    m_simState.setRawYaw(m_gyroSim.getGyroReading().getDegrees());
    m_simState.setAngularVelocityZ(m_gyroSim.getMeasuredAngularVelocity());
  }
}
