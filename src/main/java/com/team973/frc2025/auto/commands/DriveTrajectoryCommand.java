package com.team973.frc2025.auto.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.CommandOnEvent;
import com.team973.lib.util.Conversions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class DriveTrajectoryCommand extends AutoCommand {
  private final DriveController m_drive;
  private final Optional<Trajectory<SwerveSample>> m_trajectory;
  private final Optional<Alliance> m_alliance;
  private final CommandOnEvent[] m_events;

  private double m_startTime;

  public DriveTrajectoryCommand(
      DriveController drive, String trajectoryName, CommandOnEvent... events) {
    m_drive = drive;
    m_trajectory = Choreo.loadTrajectory(trajectoryName);
    m_alliance = DriverStation.getAlliance();
    m_events = events;
  }

  public void log() {}

  public void init() {
    m_startTime = Conversions.Time.getSecTime();
    m_drive.setControllerOption(DriveController.ControllerOption.DriveWithTrajectory);
  }

  public void run() {
    if (m_trajectory.isEmpty() || m_alliance.isEmpty()) {
      return;
    }

    // for (CommandOnEvent cmds : m_events) {
    //   for (EventMarker event: m_trajectory.get().get)
    // }

    m_drive.updateTrajectory(
        m_trajectory.get().sampleAt(m_startSec, m_alliance.get().equals(Alliance.Red)).get());
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
