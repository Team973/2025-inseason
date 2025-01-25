package com.team973.frc2025.subsystems.swerve;

import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.devices.LimelightHelpers;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;

public class MegaTagSupplier {
  public interface MegaTagReceiver {
    public void observeVisionData(
        String llName,
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDev);
  }

  private final String m_llName;
  private final GreyPigeon m_pigeon;
  private List<MegaTagReceiver> m_receivers = new ArrayList<MegaTagReceiver>();
  private Alliance m_alliance;
  // Until we get our seed heading and our alliance (which requires us)
  // to wait for a message from the driver station, we cannot with any
  // confidence provide pose data.
  private boolean m_allianceInitialized;
  private boolean m_headingInitialized;

  public MegaTagSupplier(String llName, GreyPigeon pigeon, Pose3d cameraPoseRobotSpace) {
    m_llName = llName;
    m_pigeon = pigeon;

    LimelightHelpers.setCameraPose_RobotSpace(
        llName,
        cameraPoseRobotSpace.getX(),
        cameraPoseRobotSpace.getY(),
        cameraPoseRobotSpace.getZ(),
        Rotation2d.fromRadians(cameraPoseRobotSpace.getRotation().getX()).getDegrees(),
        Rotation2d.fromRadians(cameraPoseRobotSpace.getRotation().getY()).getDegrees(),
        Rotation2d.fromRadians(cameraPoseRobotSpace.getRotation().getZ()).getDegrees());

    setHeading();
  }

  public void setAlliance(Alliance alliance) {
    m_alliance = alliance;
    m_allianceInitialized = true;
  }

  public void addReceiver(MegaTagReceiver newReceiver) {
    m_receivers.add(newReceiver);
  }

  private void setHeading() {
    LimelightHelpers.SetRobotOrientation(
        m_llName,
        // Yaw (heading)
        m_pigeon.getYaw().getRadians(),
        m_pigeon.getAngularVelocity().getDegrees(),
        // Pitch (hopefully not)
        0.0,
        0.0,
        // Roll (the ship is going down, captain!)
        0.0,
        0.0);
  }

  public void syncSensors() {
    setHeading();
    doCycle();
  }

  private void doCycle() {
    if (!m_allianceInitialized || !m_headingInitialized) {
      // Reject any measurements from before we set these values.
      return;
    }

    if (Math.abs(m_pigeon.getAngularVelocity().getDegrees()) > 720) {
      // llDocs strongly recommend ignoring visiond data while we are rotating quickly
      return;
    }

    LimelightHelpers.PoseEstimate mt2;
    if (m_alliance == Alliance.Red) {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(m_llName);
    } else {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_llName);
    }

    if (mt2.tagCount == 0) {
      // If there's no tags then how could we trust this measurement?
      return;
    }

    // TODO: We need to decide on what heuristics we're going to use for these. A good reference
    // here would be to look at the 2024 poofs code where they generate a value between 2.0 and
    // 0.2 based on a handful of criteria. Higher numbers mean less trustworthy samples.
    // Ref
    // github.com/Team254/FRC-2024-Public/src/main/java/com/team254/frc2024/subsystems/vision/VisionSubsystem.java
    Matrix<N3, N1> confidenceStdDev = VecBuilder.fill(2.0, 2.0, 9999999);
    for (MegaTagReceiver receiver : m_receivers) {
      receiver.observeVisionData(m_llName, mt2.pose, mt2.timestampSeconds, confidenceStdDev);
    }
  }
}
