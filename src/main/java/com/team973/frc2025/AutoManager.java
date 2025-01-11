package com.team973.frc2025;

import com.team973.frc2025.auto.modes.NoAuto;
import com.team973.frc2025.auto.modes.TaxiAuto;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Arrays;
import java.util.List;

public class AutoManager {
  private AutoMode m_currentMode;
  private final List<Auto> m_availableAutos = Arrays.asList(Auto.NoAuto, Auto.TaxiAuto);
  private int m_selectedMode = 1;

  public enum Auto {
    NoAuto,
    TaxiAuto,
  }

  private final AutoMode m_noAuto;
  private final AutoMode m_taxiAuto;

  public AutoManager(Logger logger, DriveController drive) {
    m_noAuto = new NoAuto(logger);
    m_taxiAuto = new TaxiAuto(logger.subLogger("taxi"), drive);
  }

  public void increment() {
    m_selectedMode += 1;
    if (m_selectedMode >= m_availableAutos.size()) {
      m_selectedMode = 0;
    }
  }

  public void decrement() {
    m_selectedMode -= 1;
    if (m_selectedMode < 0) {
      m_selectedMode = m_availableAutos.size() - 1;
    }
  }

  public Auto getSelectedMode() {
    return m_availableAutos.get(m_selectedMode);
  }

  public Pose2d getStartingPose() {
    return m_currentMode.getStartingPose();
  }

  public void run() {
    m_currentMode.run();
    m_currentMode.log();
  }

  public void init() {
    selectAuto(m_availableAutos.get(m_selectedMode));
    m_currentMode.init();
  }

  private void selectAuto(Auto mode) {
    switch (mode) {
      case NoAuto:
        m_currentMode = m_noAuto;
        break;
      case TaxiAuto:
        m_currentMode = m_taxiAuto;
        break;
    }
  }
}
