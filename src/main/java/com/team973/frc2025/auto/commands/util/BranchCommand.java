package com.team973.frc2025.auto.commands.util;

import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.Logger;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BranchCommand extends AutoCommand {
  private final boolean m_condition;
  private final AutoCommand m_falseCommand;
  private final AutoCommand m_trueCommand;

  public BranchCommand(
      Logger logger, boolean condition, AutoCommand falseCommand, AutoCommand trueCommand) {
    m_condition = condition;
    m_falseCommand = falseCommand;
    m_trueCommand = trueCommand;
  }

  @Override
  public void init() {
    if (m_condition) {
      m_trueCommand.init();
    } else {
      m_falseCommand.init();
    }
  }

  @Override
  public void run(Alliance alliance) {
    if (m_condition) {
      m_trueCommand.run(alliance);
    } else {
      m_falseCommand.run(alliance);
    }
  }

  @Override
  public void log(Alliance alliance) {
    if (m_condition) {
      m_trueCommand.log(alliance);
    } else {
      m_falseCommand.log(alliance);
    }
  }

  @Override
  public boolean isCompleted() {
    if (m_condition) {
      return m_trueCommand.isCompleted();
    } else {
      return m_falseCommand.isCompleted();
    }
  }

  @Override
  public void postComplete(boolean interrupted) {
    if (m_condition) {
      m_trueCommand.postComplete(interrupted);
    } else {
      m_falseCommand.postComplete(interrupted);
    }
  }
}
