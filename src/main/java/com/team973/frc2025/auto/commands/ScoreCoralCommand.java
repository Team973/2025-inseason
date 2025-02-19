package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.lib.util.AutoCommand;

public class ScoreCoralCommand extends AutoCommand {
  private final DriveController m_drive;
  private final Superstructure m_superstructure;

  private final ReefFace m_targetReefFace;
  private final ReefLevel m_targetReefLevel;

  public ScoreCoralCommand(
      DriveController drive,
      Superstructure superstructure,
      ReefFace targetReefFace,
      ReefLevel targetReefLevel) {
    m_drive = drive;
    m_superstructure = superstructure;

    m_targetReefFace = targetReefFace;
    m_targetReefLevel = targetReefLevel;
  }

  @Override
  public void init() {
    m_drive.setControllerOption(ControllerOption.DriveWithLimelight);
    m_drive.getDriveWithLimelight().setTargetReefFace(m_targetReefFace);
    m_superstructure.setTargetReefLevel(m_targetReefLevel);
    m_drive
        .getDriveWithLimelight()
        .targetReefPosition(
            () -> m_superstructure.readyToScore(), () -> m_superstructure.finishedScoring());
  }

  @Override
  public void run() {}

  @Override
  public void log() {}

  @Override
  public boolean isCompleted() {
    return m_drive.getDriveWithLimelight().getTargetingComplete();
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
