package com.team973.lib.util;

import com.team973.frc2025.auto.commands.util.SequentialCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public abstract class AutoMode extends SequentialCommand {
  private final Pose2d m_startingPoseBlue;

  public AutoMode(Logger logger, Pose2d startingPoseBlue, AutoCommand... commands) {
    super(logger, commands);
    m_startingPoseBlue = startingPoseBlue;
  }

  public Pose2d getStartingPose(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return m_startingPoseBlue;
    } else {
      return new Pose2d(
          m_startingPoseBlue.getTranslation(),
          m_startingPoseBlue.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
  }

  public abstract String getName();
}
