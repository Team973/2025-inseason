package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CANdleManger extends Subsystem.Stateless {
  public final CANdle m_candle = new CANdle(18, "rio");

  public List<ISignaler> m_priortyQue;

  public CANdleManger(Logger logger) {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);

    m_priortyQue = new ArrayList<ISignaler>();
    Collections.sort(m_priortyQue, new SignalerComaparator());
  }

  public void addSignaler(ISignaler newSignaler) {
    m_priortyQue.add(newSignaler);
    Collections.sort(m_priortyQue, new SignalerComaparator());
  }

  public void priortOder() {}

  @Override
  public void log() {}

  @Override
  public void syncSensors() {}

  @Override
  public void update() {

    for (ISignaler current : m_priortyQue) {
      if (current.isEnabled()) {
        current.update(m_candle);
        return;
      }
    }
  }

  @Override
  public void reset() {}

  private static class SignalerComaparator implements Comparator<ISignaler> {

    @Override
    public int compare(ISignaler o1, ISignaler o2) {
      return Integer.compare(o1.getPriorty(), o2.getPriorty());
    }
  }
}
