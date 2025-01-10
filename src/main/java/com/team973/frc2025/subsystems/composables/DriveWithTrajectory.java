package com.team973.frc2025.subsystems.composables;

import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.Drive;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.GreyHolonomicDriveController;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class DriveWithTrajectory implements DriveComposable {
  private ChassisSpeeds m_driveInput;
  private final GreyHolonomicDriveController m_controller;
  private Drive m_drive;
  private Trajectory m_trajectory;
  // when update trajectory was called, this is the time the caller thought we were at
  private double m_timeSeconds;
  // this is the time when the update trajectory function was called
  private double m_updateTimeStamp;
  private Rotation2d m_rot2d;
  private Rotation2d m_rotationalVelocityPerSecond;

  private final Logger m_logger;

  public DriveWithTrajectory(Logger logger, Drive drive) {
    m_controller =
        new GreyHolonomicDriveController(
            new PIDController(50.0, 0.0, 0.0), // 32
            new PIDController(50.0, 0.0, 0.0), // 32
            new ProfiledPIDController(
                8.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 7.0)));
    m_drive = drive;
    m_logger = logger;
  }

  public synchronized void updateTrajectory(
      Trajectory trajectory,
      double timeSeconds,
      Rotation2d rot2d,
      Rotation2d rotationalVelocityPerSecond) {
    m_trajectory = trajectory;
    m_timeSeconds = timeSeconds;
    m_rot2d = rot2d;
    m_updateTimeStamp = Timer.getFPGATimestamp();
    m_rotationalVelocityPerSecond = rotationalVelocityPerSecond;
  }

  @Override
  public synchronized ChassisSpeeds getOutput() {
    if (m_trajectory == null) {
      return null;
    }
    Pose2d currentPosition = m_drive.getPose();
    Trajectory.State currentTrajectoryState =
        m_trajectory.sample(m_timeSeconds + Timer.getFPGATimestamp() - m_updateTimeStamp);
    currentTrajectoryState.poseMeters =
        new Pose2d(
            currentTrajectoryState.poseMeters.getTranslation(), m_drive.getPose().getRotation());
    m_driveInput = m_controller.calculate(currentPosition, currentTrajectoryState, m_rot2d);
    m_driveInput =
        new ChassisSpeeds(
            m_driveInput.vxMetersPerSecond,
            m_driveInput.vyMetersPerSecond,
            m_driveInput.omegaRadiansPerSecond + m_rotationalVelocityPerSecond.getRadians());
    m_logger.log("Trajectory X", currentTrajectoryState.poseMeters.getX());
    m_logger.log("Trajectory Y", currentTrajectoryState.poseMeters.getY());
    m_logger.log(
        "Trajectory Rot", Math.toDegrees(m_controller.getThetaController().getGoal().position));
    m_logger.log("Current X", currentPosition.getX());
    m_logger.log("Current Y", currentPosition.getY());
    m_logger.log("Current Rot", currentPosition.getRotation().getDegrees());
    m_logger.log("Rotational Velocity", m_rotationalVelocityPerSecond.getDegrees());
    return m_driveInput;
  }
}
