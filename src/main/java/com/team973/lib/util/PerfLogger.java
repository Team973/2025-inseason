package com.team973.lib.util;

public class PerfLogger {
  private final Logger m_logger;
  private SignalAggregator m_loopTimeTracker = new SignalAggregator();

  public PerfLogger(Logger logger) {
    m_logger = logger;
  }

  public void observe(double durationSec) {
    m_loopTimeTracker.sample(durationSec);

    m_logger.log("mean duration sec", m_loopTimeTracker.getMean());
    m_logger.log("stddev", m_loopTimeTracker.getStdDev());
    m_logger.log("count", m_loopTimeTracker.samples());
  }
}
