package com.team973.lib.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import java.util.Optional;
import java.util.OptionalInt;

public class FMSData {
  private final String m_eventName;
  private final int m_matchNumber;
  private final Optional<Alliance> m_alliance;
  private final MatchType m_matchType;
  private final OptionalInt m_location;
  private final String m_gameSpecificMessage;

  public FMSData(
      String eventName,
      int matchNumber,
      Optional<Alliance> alliance,
      MatchType matchType,
      OptionalInt location,
      String gameSpecificMessage) {
    m_eventName = eventName;
    m_matchNumber = matchNumber;
    m_alliance = alliance;
    m_matchType = matchType;
    m_location = location;
    m_gameSpecificMessage = gameSpecificMessage;
  }

  public String getEventName() {
    return m_eventName;
  }

  public int getMatchNumber() {
    return m_matchNumber;
  }

  public Optional<Alliance> getAlliance() {
    return m_alliance;
  }

  public MatchType getMatchType() {
    return m_matchType;
  }

  public OptionalInt getLocation() {
    return m_location;
  }

  public String getGameSpecificMessage() {
    return m_gameSpecificMessage;
  }
}
