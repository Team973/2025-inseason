package com.team973.lib.util;

/** Base interface for all subsystems */
public interface Subsystem {

  /** Log subsystem data. Called while the robot is on. */
  public void log();

  /**
   * Update the sensors in the subsystem. This should be called before doing any calculations based
   * on the subsystem.
   */
  public void syncSensors();

  /** Update the subsystem. Call this periodically when the robot is enabled. */
  public void update();

  /** Reset the subsystem. */
  public void reset();
}
