package com.team973.frc2025.auto.commands.util;

import com.google.common.collect.ImmutableList;
import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashSet;

public class ConcurrentCommand extends AutoCommand {
  private final ImmutableList<AutoCommand> m_cmdList;
  private HashSet<AutoCommand> m_finishedCmds;
  private Double m_timeoutSec = null;

  /**
   * Constructor for concurrent command class.
   *
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public ConcurrentCommand(AutoCommand... commands) {
    this.m_cmdList = ImmutableList.copyOf(commands);
    m_finishedCmds = new HashSet<>();
  }

  /**
   * Constructor for concurrent command class with timeout parameter.
   *
   * @param timeout This sets the timeout for the commands in seconds.
   * @param commands This is the parameter for a variable amount of auto commands.
   */
  public ConcurrentCommand(double timeoutSec, AutoCommand... commands) {
    this(commands);
    this.m_timeoutSec = timeoutSec;
  }

  public void init() {
    if (m_timeoutSec != null) {
      setTargetSec(m_timeoutSec);
    }

    for (var command : m_cmdList) {
      command.init();
    }
  }

  public void run(Alliance alliance) {
    if (isCompleted()) {
      return;
    }

    for (var command : m_cmdList) {
      if (m_finishedCmds.contains(command)) {
        continue;
      }

      command.run(alliance);

      if (command.isCompleted()) {
        command.postComplete(false);
        m_finishedCmds.add(command);
      } else if (command.hasElapsed()) {
        command.postComplete(true);
        m_finishedCmds.add(command);
      }
    }
  }

  public boolean isCompleted() {
    return m_finishedCmds.size() == m_cmdList.size();
  }

  public void log(Alliance alliance) {}

  public void postComplete(boolean interrupted) {
    if (interrupted) {
      for (var command : m_cmdList) {
        if (!m_finishedCmds.contains(command)) {
          command.postComplete(true);
        }
      }
    }
  }
}
