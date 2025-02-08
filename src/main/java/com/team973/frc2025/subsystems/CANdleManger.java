package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;

public class CANdleManger implements Subsystem {
  private final Logger m_logger;
  public final CANdle m_candle = new CANdle(18, "rio");
  public BlinkingSignaler m_activeSignaler =
      new BlinkingSignaler(
          new Logger("active signaler"), RobotInfo.Colors.OFF, RobotInfo.Colors.OFF, 1000);
  public final BlinkingSignaler m_redBlinker =
      new BlinkingSignaler(
          new Logger("RedBlinking Signaler"), RobotInfo.Colors.RED, RobotInfo.Colors.OFF, 500);
  public final BlinkingSignaler m_blueBlinker =
      new BlinkingSignaler(
          new Logger("BlueBlinking Signaler"), RobotInfo.Colors.BLUE, RobotInfo.Colors.OFF, 1000);
  public final BlinkingSignaler m_offBlinker =
      new BlinkingSignaler(
          new Logger("offBlinkingSignaler"), RobotInfo.Colors.OFF, RobotInfo.Colors.OFF, 1000);

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
    m_activeSignaler.update(m_candle);
  }

  @Override
  public void reset() {}
}
