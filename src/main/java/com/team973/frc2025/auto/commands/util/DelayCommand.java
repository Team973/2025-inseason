package com.team973.frc2025.auto.commands.util;

import com.team973.lib.util.AutoCommand;

public class DelayCommand extends AutoCommand {
  private final double m_targetSec;

  /**
   * The Constructor for the Wait Command class.
   *
   * @param targetSec The target second amount to wait.
   */
  public DelayCommand(double targetSec) {
    this.m_targetSec = targetSec;
  }

  public void init() {
    setTargetSec(m_targetSec);
  }

  public void run() {}

  public void log() {}

  public boolean isCompleted() {
    return hasElapsed();
  }

  public void postComplete(boolean interrupted) {}
}
