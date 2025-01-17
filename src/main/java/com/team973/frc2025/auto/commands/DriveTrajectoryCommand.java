package com.team973.frc2025.auto.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.CommandOnEvent;
import com.team973.lib.util.Logger;
import java.util.Optional;

public class DriveTrajectoryCommand extends AutoCommand {
  private final DriveController m_drive;
  private final Optional<Trajectory<SwerveSample>> m_trajectory;
  private final CommandOnEvent[] m_events;
  private final Logger m_logger;

  public DriveTrajectoryCommand(
      Logger logger, DriveController drive, String trajectoryName, CommandOnEvent... events) {
    m_logger = logger;
    m_drive = drive;
    m_trajectory = Choreo.loadTrajectory(trajectoryName);
    m_events = events;
  }

  public void log() {}

  public void init() {
    m_drive.setControllerOption(DriveController.ControllerOption.DriveWithTrajectory);

    if (m_trajectory.isPresent()) {
      m_drive.getDriveWithTrajectory().setTrajectory(m_trajectory.get());
    }
  }

  public void run() {}

  public boolean isCompleted() {
    if (m_trajectory.isEmpty()) {
      return true;
    }

    return m_drive.getDriveWithTrajectory().isComplete();
  }

  public void postComplete(boolean interrupted) {}
}
