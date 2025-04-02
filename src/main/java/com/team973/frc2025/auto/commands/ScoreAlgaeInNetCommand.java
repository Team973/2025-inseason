package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.GamePiece;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ScoreAlgaeInNetCommand extends AutoCommand {
  private final Superstructure m_superstructure;

  public ScoreAlgaeInNetCommand(Superstructure superstructure) {
    m_superstructure = superstructure;
  }

  @Override
  public void init() {
    m_superstructure.setTargetReefLevel(ReefLevel.Net);
    m_superstructure.setGamePieceMode(GamePiece.Algae);
    m_superstructure.setState(Superstructure.State.Manual);
    m_superstructure.setManualScore(true);
    m_superstructure.setManualArmivator(true);
  }

  @Override
  public void run(Alliance alliance) {}

  @Override
  public void log(Alliance alliance) {}

  @Override
  public boolean isCompleted() {
    return !m_superstructure.getHasAlgae();
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
