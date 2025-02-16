package com.team973.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class Logger {
  private final String m_prefix;
  private final double m_secondsPerLog;

  private double m_nextAllowedLogTime = Conversions.Time.getSecTime();

  private boolean m_logAllowed = true;

  private List<Runnable> m_subLoggerUpdates = new ArrayList<>();

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

    // DogLog.setOptions(new DogLogOptions().withNtPublish(true));
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
    m_subLoggerUpdates.add(subLogger::update);

    return subLogger;
  }

  public void log(String key, double value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, String value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, int value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, boolean value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, double[] value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, String[] value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, int[] value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, boolean[] value) {
    // if (m_logAllowed) {
    //   DogLog.log(m_prefix + "/" + key, value);
    // }
  }

  public void log(String key, DoubleSupplier valueSupplier) {
    // if (m_logAllowed) {
    //   log(key, valueSupplier.getAsDouble());
    // }
  }

  public void log(String key, IntSupplier valueSupplier) {
    // if (m_logAllowed) {
    //   log(key, valueSupplier.getAsInt());
    // }
  }

  public void log(String key, BooleanSupplier valueSupplier) {
    // if (m_logAllowed) {
    //   log(key, valueSupplier.getAsBoolean());
    // }
  }

  /**
   * Updates the logger. If the time since the last log is greater than the seconds per log, then
   * logging is allowed for the current cycle.
   */
  public void update() {
    m_subLoggerUpdates.forEach((update) -> update.run());

    if (Conversions.Time.getSecTime() > m_nextAllowedLogTime) {
      m_nextAllowedLogTime += m_secondsPerLog;
      m_logAllowed = true;
      return;
    }

    m_logAllowed = false;
  }
}
