package com.team973.frc2025.auto.commands;

import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.lib.util.AutoCommand;
import com.team973.lib.util.CommandOnEvent;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

public class DriveTrajectoryCommand extends AutoCommand {
  private final DriveController m_drive;
  private final Optional<Trajectory<SwerveSample>> m_trajectory;

  private final HashMap<String, AutoCommand> m_events;

  private final List<EventMarker> m_commandList = new ArrayList<>();
  private int m_pendingEventIndex = 0;

  private AutoCommand m_currentCommand;

  public DriveTrajectoryCommand(
      DriveController drive, String trajectoryName, CommandOnEvent... events) {
    m_drive = drive;
    m_trajectory = Choreo.loadTrajectory(trajectoryName);

    m_events = new HashMap<>();

    for (CommandOnEvent event : events) {
      m_events.put(event.getEventName(), event.getCommand());
    }

    for (EventMarker marker : m_trajectory.get().events()) {
      m_commandList.add(marker);
    }

    Collections.sort(m_commandList, this::compare);

    m_currentCommand = null;
  }

  private int compare(EventMarker a, EventMarker b) {
    return Double.compare(a.timestamp, b.timestamp);
  }

  public void log() {}

  public void init() {
    m_drive.setControllerOption(DriveController.ControllerOption.DriveWithTrajectory);

    if (m_trajectory.isPresent()) {
      m_drive.getDriveWithTrajectory().setTrajectory(m_trajectory.get());
    }
  }

  public void run() {
    if (m_commandList.get(m_pendingEventIndex).timestamp
            <= m_drive.getDriveWithTrajectory().getTimeSecFromStart()
        && m_commandList.size() > 0) {
      if (m_currentCommand != null) {
        m_currentCommand.postComplete(true);
      }

      m_pendingEventIndex++;
      m_currentCommand = m_events.get(m_commandList.get(m_pendingEventIndex).event);

      m_currentCommand.init();
    }

    if (m_currentCommand.isCompleted() && m_currentCommand != null) {
      m_currentCommand.postComplete(false);
      m_currentCommand = null;
    }

    if (m_currentCommand != null) {
      m_currentCommand.run();
    }
  }

  public boolean isCompleted() {
    if (m_trajectory.isEmpty()) {
      return true;
    }

    return m_drive.getDriveWithTrajectory().isComplete();
  }

  public void postComplete(boolean interrupted) {}
}
