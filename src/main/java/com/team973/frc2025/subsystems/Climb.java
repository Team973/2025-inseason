package com.team973.frc2025.subsystems;

import com.team973.lib.devices.GreyTalonFX;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class Climb implements Subsystem {

  private final Logger m_logger;

  public final GreyTalonFX m_climb;

  public Climb(Logger logger) {
    m_logger = logger;
    m_climb = new GreyTalonFX(35, "Canivore", m_logger);
  }

  @Override
  public void log() {
    double climbPosition = m_climb.getPosition().getValueAsDouble();
    m_logger.log("Position", climbPosition);
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {}

  @Override
  public void reset() {}
}
