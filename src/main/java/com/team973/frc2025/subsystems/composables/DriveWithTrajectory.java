package com.team973.frc2025.subsystems.composables;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.Drive;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.GreyHolonomicDriveController;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.Optional;

public class DriveWithTrajectory extends DriveComposable {
  private static final SwerveSample NULL_SAMPLE =
      new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[0], new double[0]);

  private final GreyHolonomicDriveController m_controller;
  private final Logger m_logger;

  private SwerveSample m_currentSample;
  private Pose2d m_currentPose;

  private Trajectory<SwerveSample> m_trajectory;

  private double m_trajectoryStartTime = Conversions.Time.getSecTime();

  public DriveWithTrajectory(Logger logger, Drive drive) {
    m_controller =
        new GreyHolonomicDriveController(
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new ProfiledPIDController(
                2.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 7.0)));
    m_logger = logger;
    m_currentSample = null;
    m_currentPose = null;

    log();
  }

  public synchronized void updatePose(Pose2d pose) {
    m_currentPose = pose;
  }

  public synchronized void setTrajectory(Trajectory<SwerveSample> trajectory) {
    m_trajectory = trajectory;
    m_trajectoryStartTime = Conversions.Time.getSecTime();
  }

  public synchronized boolean isComplete() {
    return m_trajectory.getTotalTime() <= getTimeSecFromStart();
  }

  public double getTimeSecFromStart() {
    return Conversions.Time.getSecTime() - m_trajectoryStartTime;
  }

  public synchronized void log() {
    SwerveSample logSample;

    if (m_currentSample == null) {
      logSample = NULL_SAMPLE;
    } else {
      logSample = m_currentSample;
    }

    m_logger.log(
        "SampleLog",
        new double[] {
          logSample.x, logSample.y, logSample.heading,
        });

    m_logger.log("sample/X", logSample.x);
    m_logger.log("sample/y", logSample.y);
    m_logger.log("sample/vx", logSample.vx);
    m_logger.log("sample/vy", logSample.vy);
    m_logger.log("sample/Heading Deg", Rotation2d.fromRadians(logSample.heading).getDegrees());
    m_logger.log("sample/Omega", logSample.omega);

    m_logger.log("X Position Error", m_controller.getXController().getError());
    m_logger.log("X Velocity Error", m_controller.getXController().getErrorDerivative());
    m_logger.log("Y Position Error", m_controller.getYController().getError());
    m_logger.log("Y Velocity Error", m_controller.getYController().getErrorDerivative());
    m_logger.log("Theta Position Error", m_controller.getThetaController().getPositionError());
    m_logger.log("Theta Velocity Error", m_controller.getThetaController().getVelocityError());
  }

  public void init() {
    m_controller.getThetaController().reset(m_currentPose.getRotation().getRadians());
  }

  @Override
  public synchronized ChassisSpeeds getOutput() {
    Optional<SwerveSample> sample = m_trajectory.sampleAt(getTimeSecFromStart(), false);

    if (m_currentPose == null || sample.isEmpty()) {
      return new ChassisSpeeds();
    }

    m_currentSample = sample.get();

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
