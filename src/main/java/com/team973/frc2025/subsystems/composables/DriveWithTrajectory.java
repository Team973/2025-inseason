package com.team973.frc2025.subsystems.composables;

import choreo.trajectory.SwerveSample;
import com.team973.frc2025.subsystems.Drive;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.GreyHolonomicDriveController;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
            new PIDController(2.0, 0, 0)
            // new ProfiledPIDController(
            //     8.0,
            //     0.0,
            //     0.0,
            //     new TrapezoidProfile.Constraints(
            //         DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 7.0))
            );
    m_logger = logger;
    m_currentSample = null;
    m_currentPose = null;

    log();
  }

  public synchronized void updateTrajectory(SwerveSample sample, Pose2d pose) {
    m_currentSample = sample;
    m_currentPose = pose;
  }

  public void log() {
    m_logger.log("X Position Error", m_controller.getXController().getPositionError());
    m_logger.log("X Velocity Error", m_controller.getXController().getVelocityError());
    m_logger.log("Y Position Error", m_controller.getYController().getPositionError());
    m_logger.log("Y Velocity Error", m_controller.getYController().getVelocityError());
    m_logger.log("Theta Position Error", m_controller.getThetaController().getPositionError());
    m_logger.log("Theta Velocity Error", m_controller.getThetaController().getVelocityError());
  }

  @Override
  public synchronized ChassisSpeeds getOutput() {
    log();

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
