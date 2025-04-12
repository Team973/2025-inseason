package com.team973.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

public class AllianceCache {
  private static boolean m_allianceInitialized = false;
  private static Optional<Alliance> m_alliance;

  public static AtomicInteger m_numberOfGetCalls = new AtomicInteger(0);

  // GetCachedAlliance will keep asking the DriverStation for the
  // alliance until we get back a non-empty alliance.
  public static synchronized Optional<Alliance> Get() {
    if (m_allianceInitialized) {
      return m_alliance;
    }

    m_alliance = DriverStation.getAlliance();
    m_numberOfGetCalls.getAndIncrement();

    if (m_alliance.isPresent()) {
      m_allianceInitialized = true;
    }
    return m_alliance;
  }
}
