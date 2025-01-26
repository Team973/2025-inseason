package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class CANdleManger implements Subsystem {
  private final Logger m_logger;
  private final CANdle m_candle = new CANdle(18, "rio");

  public CANdleManger(Logger logger) {
    m_logger = logger;
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  @Override
  public void log() {}

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    m_candle.setLEDs(255, 254, 253);
  }

  @Override
  public void reset() {}
}
