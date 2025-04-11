package com.team973.frc2025.auto.commands.util;

import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.Logger;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BlockingLambdaCommand extends AutoCommand {
  private final BlockingLambda m_lambda;

  private final Logger m_logger;

  private final double m_targetSec;

  private boolean m_lastLambdaResult = false;

  public interface BlockingLambda {
    boolean get();
  }

  public BlockingLambdaCommand(BlockingLambda lambda, double targetSec, Logger logger) {
    m_lambda = lambda;
    m_targetSec = targetSec;
    m_logger = logger;
  }

  public BlockingLambdaCommand(BlockingLambda lambda, Logger logger) {
    this(lambda, 1.0, logger);
  }

  public BlockingLambdaCommand(BlockingLambda lambda, double targetSec) {
    this(lambda, targetSec, null);
  }

  @Override
  public void init() {
    setTargetSec(m_targetSec);
  }

  @Override
  public void run(Alliance alliance) {
    m_lastLambdaResult = m_lambda.get();
  }

  public void log(Alliance alliance) {}

  private void logIsComplete(String status) {
    if (m_logger != null) {
      m_logger.log("isComplete", status);
    }
  }

  @Override
  public boolean isCompleted() {
    if (m_lastLambdaResult) {
      logIsComplete("Lambda returned true");
      return true;
    } else if (hasElapsed()) {
      logIsComplete("Timeout");
      return true;
    }

    logIsComplete("False");
    return false;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
