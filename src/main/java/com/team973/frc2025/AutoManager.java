package com.team973.frc2025;

import com.team973.frc2025.auto.modes.ClawTestAuto;
import com.team973.frc2025.auto.modes.NoAuto;
import com.team973.frc2025.auto.modes.TaxiAuto;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.Logger;
import java.util.Arrays;
import java.util.List;

public class AutoManager {
  private AutoCommand m_currentMode;
  private final List<AutoMode> m_availableAutoModes =
      Arrays.asList(AutoMode.NoAuto, AutoMode.TaxiAuto, AutoMode.ClawTestAuto);
  private int m_selectedMode = 0;

  public enum AutoMode {
    NoAuto,
    TaxiAuto,
    ClawTestAuto,
  }

  private final AutoCommand m_noAuto;
  private final AutoCommand m_taxiAuto;
  private final AutoCommand m_clawTestAuto;

  public AutoManager(Logger logger, DriveController drive, Claw claw) {
    m_noAuto = new NoAuto(logger);
    m_taxiAuto = new TaxiAuto(logger.subLogger("taxi"), drive);
    m_clawTestAuto = new ClawTestAuto(logger, claw);
  }

  public void increment() {
    m_selectedMode += 1;
    if (m_selectedMode >= m_availableAutoModes.size()) {
      m_selectedMode = 0;
    }
  }

  public void decrement() {
    m_selectedMode -= 1;
    if (m_selectedMode < 0) {
      m_selectedMode = m_availableAutoModes.size() - 1;
    }
  }

  public AutoMode getSelectedMode() {
    return m_availableAutoModes.get(m_selectedMode);
  }

  public void run() {
    m_currentMode.run();
    m_currentMode.log();
  }

  public void init() {
    selectAuto(m_availableAutoModes.get(m_selectedMode));
    m_currentMode.init();
  }

  private void selectAuto(AutoMode mode) {
    switch (mode) {
      case NoAuto:
        m_currentMode = m_noAuto;
        break;
      case TaxiAuto:
        m_currentMode = m_taxiAuto;
        break;
      case ClawTestAuto:
        m_currentMode = m_clawTestAuto;
        break;
    }
  }
}
