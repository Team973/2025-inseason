package com.team973.frc2025.subsystems.states;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.GamePiece;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.util.SubsystemState;
import edu.wpi.first.wpilibj.DriverStation;

public class ScoreState implements SubsystemState {
  private final Superstructure m_superstructure;

  public ScoreState(Superstructure superstructure) {
    m_superstructure = superstructure;
  }

  public void init() {
    m_superstructure.setManualScore(false);
  }

  public void run() {
    m_superstructure.clawIntake();

    switch (m_superstructure.getDriveWithLimelightTargetStage()) {
      case MoveToApproach:
        if (m_superstructure.isDriveNearApproach()) {
          m_superstructure.armTargetReefLevel();
          m_superstructure.wristTargetReefLevel();
          m_superstructure.elevatorTargetReefLevel();

          m_superstructure.setManualArmivator(true);
        }
        break;
      case Approach:
        m_superstructure.armTargetReefLevel();
        m_superstructure.elevatorTargetReefLevel();
        m_superstructure.wristTargetReefLevel();

        m_superstructure.setManualArmivator(true);
        break;
      case MoveToScoring:
        m_superstructure.armTargetReefLevel();
        m_superstructure.elevatorTargetReefLevel();
        m_superstructure.wristTargetReefLevel();
        break;
      case Scoring:
        if (m_superstructure.getManualScore()
            && (m_superstructure.getGamePieceMode() == GamePiece.Coral
                || m_superstructure.getTargetReefLevel() == ReefLevel.Processor
                || m_superstructure.getTargetReefLevel() == ReefLevel.Net)) {
          m_superstructure.clawScore();
        }

        m_superstructure.armTargetReefLevel();
        m_superstructure.elevatorTargetReefLevel();
        m_superstructure.wristTargetReefLevel();
        break;
      case MoveToBackOff:
        if (m_superstructure.getGamePieceMode() == GamePiece.Coral) {
          m_superstructure.clawOff();
        } else {
          m_superstructure.clawIntake();
        }

        m_superstructure.armTargetReefLevel();
        m_superstructure.elevatorTargetReefLevel();
        m_superstructure.wristTargetReefLevel();
        break;
      case BackOff:
        if (m_superstructure.getGamePieceMode() == GamePiece.Coral || DriverStation.isTeleop()) {
          m_superstructure.armStow();
          m_superstructure.elevatorStow();
          m_superstructure.wristStow();

          m_superstructure.setManualArmivator(false);
        }

        break;
    }
  }
}
