package com.team973.frc2025.auto.commands.util;

import com.google.common.collect.ImmutableList;
import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SequentialCommand extends AutoCommand {
  private final ImmutableList<AutoCommand> m_cmdList;
  private boolean m_cmdNeedsInit = true;
  private int m_currentIndex = 0;
  private double m_timeoutSec = Double.MAX_VALUE;

  private final Logger m_logger;

  /**
   * Constructor for sequential command class.
   *
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public SequentialCommand(Logger logger, AutoCommand... commands) {
    this.m_cmdList = ImmutableList.copyOf(commands);
    m_logger = logger;
  }

  /**
   * Constructor for sequential command class.
   *
   * @param timeout This sets the timeout for the commands in seconds.
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public SequentialCommand(Logger logger, double timeoutSec, AutoCommand... commands) {
    this(logger, commands);
    this.m_timeoutSec = timeoutSec;
  }

  public void init() {
    setTargetSec(m_timeoutSec);
  }

  public void run() {
    if (isCompleted()) {
      return;
    }

    final AutoCommand currentCommand = m_cmdList.get(m_currentIndex);
    SmartDashboard.putNumber("Sequential Command Index", m_currentIndex);

    if (m_cmdNeedsInit) {
      currentCommand.init();
      m_cmdNeedsInit = false;
    }

    currentCommand.run();

    if (currentCommand.isCompleted()) {
      m_currentIndex++;
      m_cmdNeedsInit = true;
      currentCommand.postComplete(false);
    } else if (currentCommand.hasElapsed()) {
      m_currentIndex++;
      m_cmdNeedsInit = true;
      currentCommand.postComplete(true);
    }
  }

  public void log() {
    m_logger.log("Index", m_currentIndex);
  }

  public boolean isCompleted() {
    return m_currentIndex >= m_cmdList.size();
  }

  public void postComplete(boolean interrupted) {}
}
