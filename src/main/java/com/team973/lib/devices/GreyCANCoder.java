package com.team973.lib.devices;

import com.ctre.phoenix6.hardware.CANcoder;
import com.team973.lib.util.Logger;

public class GreyCANCoder extends CANcoder {
  private final Logger m_logger;

  public GreyCANCoder(int deviceId, Logger logger) {
    this(deviceId, "", logger);
  }

  public GreyCANCoder(int deviceId, String canbus, Logger logger) {
    super(deviceId, canbus);
    m_logger = logger;
  }

  public void log() {
    m_logger.log("Absoltue Position in Rotations", () -> this.getAbsolutePosition().getValue().magnitude());
    m_logger.log(
        "Absoltue Position in Degrees", () -> this.getAbsolutePosition().getValue().magnitude() * 360.0);
    // m_logger.log("Supply Voltage", () -> this.getSupplyVoltage().getValue().magnitude());
    // m_logger.log("Velocity", () -> this.getVelocity().getValue().magnitude());
  }
}
