package com.team973.lib.util;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class Logger {
  private final String m_prefix;
  private final double m_secondsPerLog;
  private HashMap<String, Double> m_keyLastLoggedAt = new HashMap<String, Double>();

  /**
   * Creates an instance of the logger.
   *
   * @param prefix The NetworkTables prefix for the logger. Displayed as a folder in AdvantageScope.
   * @param secondsPerLog The amount of time in seconds after logging before the next log is
   *     allowed. This allows for down sampling to reduce loop overruns. If set to 0.0, then logging
   *     is allowed every robot cycle.
   */
  public Logger(String prefix, double secondsPerLog) {
    m_prefix = prefix;
    m_secondsPerLog = secondsPerLog;

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

  public String getPrefix() {
    return m_prefix;
  }

  private boolean isLogAllowed(String key) {
    if (m_secondsPerLog == 0) {
      return true;
    }
    double now = Conversions.Time.getSecTime();
    if (m_keyLastLoggedAt.containsKey(key) == false) {
      m_keyLastLoggedAt.put(key, now);
      return true;
    }
    double lastLoggedAt = m_keyLastLoggedAt.get(key);
    if (lastLoggedAt + m_secondsPerLog < now) {
      m_keyLastLoggedAt.put(key, now);
      return true;
    }
    return false;
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
    Logger subLogger = new Logger(m_prefix + "/" + prefix, secondsPerLog);
    return subLogger;
  }

  public void log(String key, double value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, String value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, int value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, boolean value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, double[] value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, String[] value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, int[] value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, boolean[] value) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, value);
    }
  }

  public void log(String key, DoubleSupplier valueSupplier) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, valueSupplier.getAsDouble());
    }
  }

  public void log(String key, IntSupplier valueSupplier) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, valueSupplier.getAsInt());
    }
  }

  public void log(String key, BooleanSupplier valueSupplier) {
    if (isLogAllowed(key)) {
      DogLog.log(m_prefix + "/" + key, valueSupplier.getAsBoolean());
    }
  }
}
