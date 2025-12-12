package com.team973.frc2025.subsystems.states;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.GamePiece;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.util.State;
import edu.wpi.first.wpilibj.DriverStation;

public class Score extends State {
  public Score(Superstructure superstructure) {
    super(superstructure);
  }

  public void onEntrance() {
    getSuperstructure().setManualScore(false);
  }

  public void run() {
    getSuperstructure().clawIntake();

    switch (getSuperstructure().getDriveWithLimelightTargetStage()) {
      case MoveToApproach:
        if (getSuperstructure().isDriveNearApproach()) {
          getSuperstructure().armTargetReefLevel();
          getSuperstructure().wristTargetReefLevel();
          getSuperstructure().elevatorTargetReefLevel();

          getSuperstructure().setManualArmivator(true);
        }
        break;
      case Approach:
        getSuperstructure().armTargetReefLevel();
        getSuperstructure().elevatorTargetReefLevel();
        getSuperstructure().wristTargetReefLevel();

        getSuperstructure().setManualArmivator(true);
        break;
      case MoveToScoring:
        getSuperstructure().armTargetReefLevel();
        getSuperstructure().elevatorTargetReefLevel();
        getSuperstructure().wristTargetReefLevel();
        break;
      case Scoring:
        if (getSuperstructure().getManualScore()
            && (getSuperstructure().getGamePieceMode() == GamePiece.Coral
                || getSuperstructure().getTargetReefLevel() == ReefLevel.Processor
                || getSuperstructure().getTargetReefLevel() == ReefLevel.Net)) {
          getSuperstructure().clawScore();
        }

        getSuperstructure().armTargetReefLevel();
        getSuperstructure().elevatorTargetReefLevel();
        getSuperstructure().wristTargetReefLevel();
        break;
      case MoveToBackOff:
        if (getSuperstructure().getGamePieceMode() == GamePiece.Coral) {
          getSuperstructure().clawOff();
        } else {
          getSuperstructure().clawIntake();
        }

        getSuperstructure().armTargetReefLevel();
        getSuperstructure().elevatorTargetReefLevel();
        getSuperstructure().wristTargetReefLevel();
        break;
      case BackOff:
        if (getSuperstructure().getGamePieceMode() == GamePiece.Coral || DriverStation.isTeleop()) {
          getSuperstructure().armStow();
          getSuperstructure().elevatorStow();
          getSuperstructure().wristStow();

          getSuperstructure().setManualArmivator(false);
        }

        break;
    }
  }

  public void onExit() {}
}
