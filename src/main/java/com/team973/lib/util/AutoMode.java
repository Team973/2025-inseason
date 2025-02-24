package com.team973.lib.util;

import com.team973.frc2025.auto.commands.util.SequentialCommand;
import edu.wpi.first.math.geometry.Pose2d;

public abstract class AutoMode extends SequentialCommand {
  private final Pose2d m_startingPoseBlue;

  public AutoMode(Logger logger, Pose2d startingPoseBlue, AutoCommand... commands) {
    super(logger, commands);
    m_startingPoseBlue = startingPoseBlue;
  }

  public Pose2d getStartingPose() {
    return m_startingPoseBlue;
  }

  public abstract String getName();
}
