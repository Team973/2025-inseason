package com.team973.lib.util;

/** Abstract Class for auto commands. */
public abstract class AutoCommand {
  protected double m_targetSec = Double.MAX_VALUE;
  protected double m_startSec = 0.0;

  /**
   * Checks if the target time (safety timeout) has elapsed.
   *
   * @return True if the target time has passed
   */
  public boolean hasElapsed() {
    return Conversions.Time.getSecTime() - m_startSec >= m_targetSec;
  }

  /** Initialize the auto command. */
  public abstract void init();

  /** Run the auto command. */
  public abstract void run();

  public abstract void log();

  /**
   * Check if the auto command is completed.
   *
   * @return True if the command is completed.
   */
  public abstract boolean isCompleted();

  /** Executes once after isCompleted returns true */
  public abstract void postComplete(boolean interrupted);

  /**
   * Sets the target time (safety timeout) for the command.
   *
   * @param target The target time for the command.
   */
  public void setTargetSec(double targetSec) {
    m_targetSec = targetSec;
    m_startSec = Conversions.Time.getSecTime();
  }
}
