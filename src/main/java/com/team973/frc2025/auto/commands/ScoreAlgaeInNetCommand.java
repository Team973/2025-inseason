package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.GamePiece;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.TargetStage;
import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ScoreAlgaeInNetCommand extends AutoCommand {
  private final DriveController m_drive;
  private final Superstructure m_superstructure;

  public ScoreAlgaeInNetCommand(DriveController drive, Superstructure superstructure) {
    m_drive = drive;
    m_superstructure = superstructure;
  }

  @Override
  public void init() {
    m_drive.setControllerOption(ControllerOption.DriveWithLimelight);
    m_drive.getDriveWithLimelight().setTargetReefFace(ReefFace.Net);

    m_superstructure.setTargetReefLevel(ReefLevel.Net);
    m_superstructure.setGamePieceMode(GamePiece.Algae);
    m_superstructure.setState(Superstructure.State.Score);

    m_drive.getDriveWithLimelight().targetReefPosition();
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
