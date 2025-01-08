package com.team973.lib.util;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Logger {
  private final String m_prefix;

  public Logger(String prefix) {
    m_prefix = prefix;

    DogLog.setOptions(new DogLogOptions().withNtPublish(true));
    // TODO: solve why this doesn't work
    // DogLog.setPdh(new PowerDistribution());
  }

  public String getPrefix() {
    return m_prefix;
  }

  public Logger subLogger(String prefix) {
    return new Logger(m_prefix + "/" + prefix);
  }

  public void log(String key, double value) {
    DogLog.log(m_prefix + "/" + key, value);
  }

  public void log(String key, String value) {
    DogLog.log(m_prefix + "/" + key, value);
  }

  public void log(String key, int value) {
    DogLog.log(m_prefix + "/" + key, value);
  }

  public void log(String key, boolean value) {
    DogLog.log(m_prefix + "/" + key, value);
  }

  public void log(String key, double[] value) {
    DogLog.log(m_prefix + "/" + key, value);
  }

  public void log(String key, String[] value) {
    DogLog.log(m_prefix + "/" + key, value);
  }

  public void log(String key, int[] value) {
    DogLog.log(m_prefix + "/" + key, value);
  }

  public void log(String key, boolean[] value) {
    DogLog.log(m_prefix + "/" + key, value);
  }
}
