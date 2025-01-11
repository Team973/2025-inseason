package com.team973.frc2025.subsystems.composables;

import choreo.trajectory.SwerveSample;
import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.Drive;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.GreyHolonomicDriveController;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveWithTrajectory implements DriveComposable {
  private final GreyHolonomicDriveController m_controller;
  private final Logger m_logger;

  private SwerveSample m_currentSample;
  private Pose2d m_currentPose;

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
    m_logger = logger;
    m_currentSample = null;
    m_currentPose = null;
  }

  public synchronized void updateTrajectory(SwerveSample sample, Pose2d pose) {
    m_currentSample = sample;
    m_currentPose = pose;
  }

  @Override
  public synchronized ChassisSpeeds getOutput() {
    if (m_currentPose == null || m_currentSample == null) {
      return new ChassisSpeeds();
    }

    return new ChassisSpeeds(
        m_currentSample.vx
            + m_controller.getXController().calculate(m_currentPose.getX(), m_currentSample.x),
        m_currentSample.vy
            + m_controller.getYController().calculate(m_currentPose.getY(), m_currentSample.y),
        m_currentSample.omega
            + m_controller
                .getThetaController()
                .calculate(m_currentPose.getRotation().getRadians(), m_currentSample.heading));
  }
}
