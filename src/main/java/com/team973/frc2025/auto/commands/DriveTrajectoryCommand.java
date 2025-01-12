package com.team973.frc2025.auto.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.CommandOnEvent;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;

public class DriveTrajectoryCommand extends AutoCommand {
  private final DriveController m_drive;
  private final Optional<Trajectory<SwerveSample>> m_trajectory;
  private final CommandOnEvent[] m_events;
  private final Logger m_logger;

  private double m_startTime;

  public DriveTrajectoryCommand(
      Logger logger, DriveController drive, String trajectoryName, CommandOnEvent... events) {
    m_logger = logger;
    m_drive = drive;
    m_trajectory = Choreo.loadTrajectory(trajectoryName);
    m_events = events;

    m_logger.log("X Pos", 0.0);
    m_logger.log("Y Pos", 0.0);
    m_logger.log("Heading Deg", 0.0);
  }

  public void log() {
    SwerveSample sample = m_trajectory.get().sampleAt(getTimeSecFromStart(), false).get();

    m_logger.log("X Pos", sample.x);
    m_logger.log("Y Pos", sample.y);
    m_logger.log("Heading Deg", Rotation2d.fromRadians(sample.heading).getDegrees());
  }

  public void init() {
    m_startTime = Conversions.Time.getSecTime();
    m_drive.setControllerOption(DriveController.ControllerOption.DriveWithTrajectory);
  }

  public void run() {
    if (m_trajectory.isEmpty()) {
      return;
    }

    // for (CommandOnEvent cmds : m_events) {
    //   for (EventMarker event: m_trajectory.get().get)
    // }

    m_drive.updateTrajectory(m_trajectory.get().sampleAt(getTimeSecFromStart(), false).get());
  }

  public boolean isCompleted() {
    if (m_trajectory.isEmpty()) {
      return true;
    }
    return m_trajectory.get().getTotalTime() <= getTimeSecFromStart();
  }

  public void postComplete(boolean interrupted) {}

  public double getTimeSecFromStart() {
    return Conversions.Time.getSecTime() - m_startTime;
  }
}
