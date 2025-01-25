package com.team973.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class TargetPositionRelativeToAprilTag {
  private final AprilTag m_tag;
  private final double m_initialDist;
  private final double m_finalDist;
  private final Rotation2d m_targetAngle;

  public TargetPositionRelativeToAprilTag(
      AprilTag tag, double initialDist, double finalDist, Rotation2d targetAngle) {
    m_tag = tag;
    m_initialDist = initialDist;
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

  public double getInitialDist() {
    return m_initialDist;
  }

  public double getFinalDist() {
    return m_finalDist;
  }

  public Rotation2d getTargetAngle() {
    return m_targetAngle;
  }
}
