package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class CANdleManger implements Subsystem {
  private final Logger m_logger;
  private final CANdle m_candle = new CANdle(18, "rio");
  private final BlinkingSignaler m_redBlinker =
      new BlinkingSignaler(
          new Logger("Blinking Signaler"), RobotInfo.Colors.RED, RobotInfo.Colors.OFF, 500);
  private final BlinkingSignaler m_blueBlinker =
      new BlinkingSignaler(
          new Logger("Blinking Signaler"), RobotInfo.Colors.BLUE, RobotInfo.Colors.OFF, 1000);
  private double BLINKPEIOD_MS = 333;

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
    m_blueBlinker.setCandle(m_candle).update();
  }

  @Override
  public void reset() {}
}
