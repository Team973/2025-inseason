package com.team973.lib.util;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Logger {
  private final String m_prefix;
  private final double m_secondsPerLog;

  private double m_nextAllowedLogTime;

  public Logger(String prefix, double secondsPerLog) {
    m_prefix = prefix;
    m_secondsPerLog = secondsPerLog;
    m_nextAllowedLogTime = secondsPerLog;

    DogLog.setOptions(new DogLogOptions().withNtPublish(true));
    // TODO: solve why this doesn't work
    // DogLog.setPdh(new PowerDistribution());
  }

  public Logger(String prefix) {
    this(prefix, 0.0);
  }

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

  public Logger subLogger(String prefix) {
    return new Logger(m_prefix + "/" + prefix, m_secondsPerLog);
  }

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
