package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import java.util.ArrayList;
import java.util.List;

public class CANdleManger implements Subsystem {
  private final Logger m_logger;
  public final CANdle m_candle = new CANdle(18, "rio");
  public final BlinkingSignaler m_redBlinker =
      new BlinkingSignaler(
          new Logger("RedBlinking Signaler"), RobotInfo.Colors.RED, RobotInfo.Colors.GREEN, 500);
  public final BlinkingSignaler m_blueBlinker =
      new BlinkingSignaler(
          new Logger("BlueBlinking Signaler"), RobotInfo.Colors.BLUE, RobotInfo.Colors.OFF, 1000);
  public final SolidSignaler m_offBlinker = new SolidSignaler(RobotInfo.Colors.OFF, 10);

  private final List<ISignaler> m_priorityList;

  public CANdleManger(Logger logger) {
    m_logger = logger;
    m_offBlinker.setEnabled(true);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);

    m_priorityList = new ArrayList<ISignaler>();
    m_priorityList.add(m_redBlinker);
    m_priorityList.add(m_blueBlinker);
    m_priorityList.add(m_offBlinker);
  }

  @Override
  public void log() {}

  @Override
  public void syncSensors() {}

  @Override
  public void update() {

    if (m_redBlinker.isEnabled()) {
      m_redBlinker.update(m_candle);
    } else if (m_blueBlinker.isEnabled()) {
      m_blueBlinker.update(m_candle);
    } else if (m_offBlinker.isEnabled()) {
      m_offBlinker.update(m_candle);
    }
  }

  @Override
  public void reset() {}
}
