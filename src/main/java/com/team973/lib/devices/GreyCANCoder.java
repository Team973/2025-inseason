package com.team973.lib.devices;

import com.ctre.phoenix6.hardware.CANcoder;
import com.team973.lib.util.Logger;

public class GreyCANCoder extends CANcoder {

  public GreyCANCoder(int deviceId, Logger logger) {
    this(deviceId, "", logger);
  }

  public GreyCANCoder(int deviceId, String canbus, Logger logger) {
    super(deviceId, canbus);
    }

  public void log() {
    // m_logger.log("Position in Rotations", () -> this.getPosition().getValue().magnitude());
    m_logger.log("Position in Degrees", () -> this.getPosition().getValue().magnitude() * 360.0);
    // m_logger.log("Supply Voltage", () -> this.getSupplyVoltage().getValue().magnitude());
    // m_logger.log("Velocity", () -> this.getVelocity().getValue().magnitude());
  }
}
