package com.team973.frc2025.subsystems.swerve;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.swerve.MegaTagSupplier.MegaTagReceiver;
import com.team973.frc2025.subsystems.swerve.OdometrySupplier.OdometryReceiver;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class GreyPoseEstimator implements OdometryReceiver, MegaTagReceiver {

  private SwerveDrivePoseEstimator m_poseEstimator;
  private final GreyPigeon m_pigeon;
  private Pose2d m_lastPoseMeters;
  private final OdometrySupplier m_odometrySupplier;
  private final Logger m_logger;

  private DriveController m_driveController;

  public GreyPoseEstimator(
      GreyPigeon pigeon,
      DriveController m_DriveController,
      OdometrySupplier odometrySupplier,
      Logger logger) {

    m_pigeon = pigeon;
    m_driveController = m_DriveController;
    odometrySupplier.addReceiver(this);
    m_odometrySupplier = odometrySupplier;
    m_logger = logger;
  }

  public Pose2d getPoseMeters() {
    return m_poseEstimator.getEstimatedPosition();
  }

  // TODO: This isn't going to be accurate the way we do it now. We should instead
  // just get the velocity from the odometry implementation and drop the rest.
  public Translation2d getVelocityMetersPerSeconds() {
    return m_lastPoseMeters
        .minus(getPoseMeters())
        .getTranslation()
        .times(1.0 / RobotInfo.DriveInfo.STATUS_SIGNAL_FREQUENCY);
  }

  public synchronized void resetPosition(Pose2d pose) {
    m_poseEstimator.resetPosition(m_pigeon.getYaw(), m_odometrySupplier.getPositions(), pose);
    log();
  }

  @Override
  public synchronized void observeOdometryData(
      double sampleTime, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    m_poseEstimator.updateWithTime(sampleTime, gyroAngle, modulePositions);
    m_lastPoseMeters = m_poseEstimator.getEstimatedPosition();
    m_driveController.syncSensors();
    m_driveController.update();
  }

  @Override
  public synchronized void observeVisionData(
      String llName,
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDev) {
    m_poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDev);
  }

  public synchronized void log() {
    Pose2d pose = getPoseMeters();
    m_logger.log(
        "pose",
        new double[] {
          pose.getTranslation().getX(),
          pose.getTranslation().getY(),
          pose.getRotation().getDegrees()
        });
  }
}
