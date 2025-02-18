package com.team973.lib.util;

public class SignalAggregator {
  private double m_count;
  private double m_mean;
  private double m_m2;

  public SignalAggregator() {}

  public void sample(double x) {
    m_count++;
    double delta = x - m_mean;
    m_mean += delta / m_count;
    double delta2 = x - m_mean;
    m_m2 += delta * delta2;
  }

  public double getMean() {
    return m_mean;
  }

  public double getStdDev() {
    return Math.sqrt(m_m2 / m_count);
  }

  public double samples() {
    return m_count;
  }
}
