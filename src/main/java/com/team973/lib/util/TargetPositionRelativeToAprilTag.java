package com.team973.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
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

  private Pose3d getAprilTagPose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return m_tag.getPoseFromAlliance(alliance.get());
    }

    return m_tag.getPoseFromAlliance(Alliance.Red);
  }

  public Pose2d getApproachPose() {
    return new Pose2d(
        getAprilTagPose()
            .getTranslation()
            .toTranslation2d()
            .plus(
                new Translation2d(
                    m_initialTarget.getX(),
                    getAprilTagPose()
                        .getRotation()
                        .toRotation2d()
                        .plus(Rotation2d.fromDegrees(90.0))))
            .plus(
                new Translation2d(
                    m_initialTarget.getY(), getAprilTagPose().getRotation().toRotation2d())),
        getAprilTagPose()
            .getRotation()
            .toRotation2d()
            .plus(Rotation2d.fromDegrees(180))
            .plus(m_targetAngle));
  }

  public Pose2d getScoringPose() {
    return new Pose2d(
        getAprilTagPose()
            .getTranslation()
            .toTranslation2d()
            .plus(
                new Translation2d(
                    m_initialTarget.getX(),
                    getAprilTagPose()
                        .getRotation()
                        .toRotation2d()
                        .plus(Rotation2d.fromDegrees(90.0))))
            .plus(new Translation2d(m_finalDist, getAprilTagPose().getRotation().toRotation2d())),
        getAprilTagPose()
            .getRotation()
            .toRotation2d()
            .plus(Rotation2d.fromDegrees(180))
            .plus(m_targetAngle));
  }
}
