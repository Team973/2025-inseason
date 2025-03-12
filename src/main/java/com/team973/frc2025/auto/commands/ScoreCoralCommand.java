package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.GamePiece;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefSide;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.TargetStage;
import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ScoreCoralCommand extends AutoCommand {
  private final DriveController m_drive;
  private final Superstructure m_superstructure;

  private final ReefFace m_targetReefFace;
  private final ReefSide m_targetReefSide;
  private final ReefLevel m_targetReefLevel;

  public ScoreCoralCommand(
      DriveController drive,
      Superstructure superstructure,
      ReefFace targetReefFace,
      ReefLevel targetReefLevel,
      ReefSide targetReefSide) {
    m_drive = drive;
    m_superstructure = superstructure;

    m_targetReefFace = targetReefFace;
    m_targetReefSide = targetReefSide;
    m_targetReefLevel = targetReefLevel;
  }

  @Override
  public void init() {
    m_drive.setControllerOption(ControllerOption.DriveWithLimelight);
    m_drive.getDriveWithLimelight().setTargetReefFace(m_targetReefFace);
    m_drive.getDriveWithLimelight().setTargetSide(m_targetReefSide);

    m_superstructure.setTargetReefLevel(m_targetReefLevel);
    m_superstructure.setGamePieceMode(GamePiece.Coral);
    m_superstructure.setState(Superstructure.State.Score);

    m_drive
        .getDriveWithLimelight()
        .targetReefPosition(
            () -> m_superstructure.readyToScore(), () -> m_superstructure.readyToMoveToBackOff());
  }

  @Override
  public void run(Alliance alliance) {
		if (m_drive.getDriveWithLimelight().getTargetStage() == TargetStage.Scoring) {
			m_superstructure.setManualScore(true);
		}
	}

  @Override
  public void log(Alliance alliance) {}

  @Override
  public boolean isCompleted() {
    return m_drive.getDriveWithLimelight().getTargetStage() == TargetStage.BackOff;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
