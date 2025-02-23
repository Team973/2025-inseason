package com.team973.frc2025.auto.commands.util;

import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BlockingLambdaCommand extends AutoCommand {
  private final BlockingLambda m_lambda;
  private boolean m_lastLambdaResult = false;
  private final double m_targetSec;

  public interface BlockingLambda {
    boolean get();
  }

  public BlockingLambdaCommand(BlockingLambda lambda) {
    this(lambda, 1.0);
  }

  public BlockingLambdaCommand(BlockingLambda lambda, double targetSec) {
    m_lambda = lambda;
    m_targetSec = targetSec;
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

  @Override
  public boolean isCompleted() {
    return m_lastLambdaResult || hasElapsed();
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
