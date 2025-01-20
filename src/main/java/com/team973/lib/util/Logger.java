package com.team973.lib.util;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Logger {
  private final String m_prefix;
  private final double m_secondsPerLog;

  private double m_nextAllowedLogTime;

  /**
   * Creates an instance of the logger.
   *
   * @param prefix The NetworkTables prefix for the logger. Displayed as a folder in AdvantageScope.
   * @param secondsPerLog The amount of time in seconds after logging before the next log is
   *     allowed. This allows for down sampling to reduce loop overruns.
   */
  public Logger(String prefix, double secondsPerLog) {
    m_prefix = prefix;
    m_secondsPerLog = secondsPerLog;
    m_nextAllowedLogTime = secondsPerLog;

    DogLog.setOptions(new DogLogOptions().withNtPublish(true));
    // TODO: solve why this doesn't work
    // DogLog.setPdh(new PowerDistribution());
  }

  /**
   * Creates an instance of the logger with a default of 0.0 seconds per log.
   *
   * @param prefix The NetworkTables prefix for the logger. Displayed as a folder in AdvantageScope.
   */
  public Logger(String prefix) {
    this(prefix, 0.0);
  }

  /**
   * Checks if the current time is past the allowed log time. If so, then the next allowed log time
   * is increased by the seconds per log.
   *
   * @return Whether the current time is past the allowed log time.
   */
  private boolean logAllowed() {
    if (Conversions.Time.getSecTime() > m_nextAllowedLogTime) {
      m_nextAllowedLogTime += m_secondsPerLog;
      return true;
    }

    return false;
  }

  public String getPrefix() {
    return m_prefix;
  }

  /**
   * Creates a new logger instance. The seconds per log parameter is the same as the parent logger.
   *
   * @param prefix The new prefix to append to the prefix of the parent logger. Displayed as a
   *     subfolder in AdvantageScope.
   * @return The new logger.
   */
  public Logger subLogger(String prefix) {
    return subLogger(prefix, m_secondsPerLog);
  }

  /**
   * Creates a new logger instance.
   *
   * @param prefix The new prefix to append to the prefix of the parent logger. Displayed as a
   *     subfolder in AdvantageScope.
   * @param secondsPerLog The amount of time in seconds after logging before the next log is
   *     allowed. This allows for down sampling to reduce loop overruns.
   * @return The new logger.
   */
  public Logger subLogger(String prefix, double secondsPerLog) {
    return new Logger(m_prefix + "/" + prefix, secondsPerLog);
  }

  public void log(String key, double value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, String value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, int value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, boolean value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, double[] value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, String[] value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, int[] value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, boolean[] value) {
    if (logAllowed()) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }
}
