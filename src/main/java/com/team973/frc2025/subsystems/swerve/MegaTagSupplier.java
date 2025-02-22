package com.team973.frc2025.subsystems.swerve;

import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.devices.LimelightHelpers;
import com.team973.lib.util.Logger;
import com.team973.lib.util.PerfLogger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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
  private final Logger m_logger;
  private final Logger m_perfLogger;
  private PerfLogger m_syncSensorsLogger;
  // Until we get our seed heading and our alliance (which requires us)
  // to wait for a message from the driver station, we cannot with any
  // confidence provide pose data.
  private boolean m_allianceInitialized;

  public MegaTagSupplier(
      String llName, GreyPigeon pigeon, Pose3d cameraPoseRobotSpace, Logger logger) {
    m_llName = llName;
    m_pigeon = pigeon;
    m_logger = logger;
    m_perfLogger = logger.subLogger("perf", 0.25);
    m_syncSensorsLogger = new PerfLogger(m_perfLogger.subLogger("syncSensors", 0.25));

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

  public void addReceiver(MegaTagReceiver newReceiver) {
    m_receivers.add(newReceiver);
  }

  private void setHeading() {
    LimelightHelpers.SetRobotOrientation(
        m_llName,
        // Yaw (heading)
        m_pigeon.getYaw().plus(Rotation2d.fromDegrees(0)).getDegrees(),
        m_pigeon.getAngularVelocity().getDegrees(),
        // Pitch (hopefully not)
        0.0,
        0.0,
        // Roll (the ship is going down, captain!)
        0.0,
        0.0);
  }

  public void syncSensors() {
    double startTime = Timer.getFPGATimestamp();
    setHeading();
    doCycle();
    m_syncSensorsLogger.observe(Timer.getFPGATimestamp() - startTime);
  }

  public void log() {
    m_perfLogger.log("rejected needs init", () -> getRejectedNeedsInit());
    m_perfLogger.log("rejected too spinny", () -> getRejectedTooSpinny());
    m_perfLogger.log("rejected no tags", () -> getRejectedNoTags());
    m_perfLogger.log("data received", () -> getDataReceived());
  }

  private synchronized double getRejectedNeedsInit() {
    return m_rejectedNeedsInit;
  }

  private synchronized double getRejectedTooSpinny() {
    return m_rejectedTooSpinny;
  }

  private synchronized double getRejectedNoTags() {
    return m_rejectedNoTags;
  }

  private synchronized double getDataReceived() {
    return m_dataReceived;
  }

  private double m_rejectedNeedsInit, m_rejectedTooSpinny, m_rejectedNoTags, m_dataReceived;

  private void maybeInitAlliance() {
    if (m_allianceInitialized) {
      return;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (!alliance.isPresent()) {
      return;
    }
    m_allianceInitialized = true;
    m_alliance = alliance.get();
  }

  public synchronized void doCycle() {
    maybeInitAlliance();
    if (!m_allianceInitialized) {
      // Reject any measurements from before we set these values.
      m_rejectedNeedsInit++;
      return;
    }

    // TODO: The pigeon needs to make an alliance-informed decision here :(
    if (Math.abs(m_pigeon.getAngularVelocity().getDegrees()) > 720) {
      // llDocs strongly recommend ignoring visiond data while we are rotating quickly
      m_rejectedTooSpinny++;
      return;
    }

    LimelightHelpers.PoseEstimate mt2;
    if (m_alliance == Alliance.Red) {
      // TODO: This doesn't match the limelight spec --- gotta figure out
      // why this is wrong.  It at least appears to br working on the red
      // alliance though.
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_llName);
    } else {
      mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_llName);
    }

    if (mt2 == null) {
      m_rejectedNoTags++;
      return;
    }

    if (mt2.tagCount == 0) {
      // If there's no tags then how could we trust this measurement?
      m_rejectedNoTags++;
      return;
    }

    // The following is a sequence of heuristics tuned by trial an error. The starting point
    // was 254's 2024 code. We look at a handful of factors to decide what level of confidence
    // to ascribe to the limelight reading. Higher numbers mean less confidence. The general
    // range we have seen in other teams is between 0.2 and 2.0 trust.
    // Ref
    // github.com/Team254/FRC-2024-Public/src/main/java/com/team254/frc2024/subsystems/vision/VisionSubsystem.java
    double xyStdev = 2.0;

    if (mt2.tagCount >= 2 && mt2.avgTagArea > 0.1) {
      xyStdev = 0.5;
    } else if (mt2.tagCount >= 2 || mt2.avgTagArea > 0.1) {
      xyStdev = 0.75;
    } else if (mt2.avgTagArea > 0.8) {
      xyStdev = 0.8;
    }
    Matrix<N3, N1> confidenceStdDev = VecBuilder.fill(xyStdev, xyStdev, 9999999);
    for (MegaTagReceiver receiver : m_receivers) {
      receiver.observeVisionData(m_llName, mt2.pose, mt2.timestampSeconds, confidenceStdDev);
    }
    m_dataReceived++;
  }
}
