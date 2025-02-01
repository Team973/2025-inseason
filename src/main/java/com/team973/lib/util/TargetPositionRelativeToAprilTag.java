package com.team973.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class TargetPositionRelativeToAprilTag {
  private final AprilTag m_tag;
  private final Translation2d m_initialTarget;
  private final double m_finalDist;
  private final Rotation2d m_targetAngle;

  public TargetPositionRelativeToAprilTag(
      AprilTag tag, Translation2d initialTarget, double finalDist, Rotation2d targetAngle) {
    m_tag = tag;
    m_initialTarget = initialTarget;
    m_finalDist = finalDist;
    m_targetAngle = targetAngle;
  }

  public int getAprilTagID() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return m_tag.getIDFromAlliance(alliance.get());
    }

    return m_tag.getIDFromAlliance(Alliance.Blue);
  }

  public Pose3d getAprilTagPose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return m_tag.getPoseFromAlliance(alliance.get());
    }

    return m_tag.getPoseFromAlliance(Alliance.Blue);
  }

  public Translation2d getInitialTarget() {
    return m_initialTarget;
  }

  public double getFinalDist() {
    return m_finalDist;
  }

  public Rotation2d getTargetAngle() {
    return m_targetAngle;
  }
}
