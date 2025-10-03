package com.team973.frc2025.subsystems.composables;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Drive;
import com.team973.lib.util.AllianceCache;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.GreyHolonomicDriveController;
import com.team973.lib.util.Logger;
import com.team973.lib.util.PerfLogger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class DriveWithTrajectory extends DriveComposable {
  private static final SwerveSample NULL_SAMPLE =
      new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[0], new double[0]);

  private final RobotInfo.DriveInfo m_driveInfo;

  private final GreyHolonomicDriveController m_controller;
  private final Logger m_logger;

  private SwerveSample m_currentSample;

  private Trajectory<SwerveSample> m_trajectory;

  private double m_trajectoryStartTime = Conversions.Time.getSecTime();

  private final PerfLogger m_initPerfLogger;
  private final PerfLogger m_getOutputPerfLogger;
  private final PerfLogger m_findSamplePerfLogger;
  private final PerfLogger m_getAllianceCachePerfLogger;
  private final PerfLogger m_getAlliancePerfLogger;
  private final PerfLogger m_findSecFromStartPerfLogger;
  private final PerfLogger m_getSamplePerfLogger;
  private final PerfLogger m_findChassisSpeedsPerfLogger;

  public DriveWithTrajectory(Logger logger, Drive drive) {
    m_driveInfo = RobotConfig.get().DRIVE_INFO;
    m_controller =
        new GreyHolonomicDriveController(
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new ProfiledPIDController(
                2.0,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                    m_driveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 7.0)));
    m_logger = logger;
    m_initPerfLogger = new PerfLogger(logger.subLogger("init"));
    m_getOutputPerfLogger = new PerfLogger(logger.subLogger("getOutput"));
    m_findSamplePerfLogger = new PerfLogger(logger.subLogger("findSample"));
    m_getAllianceCachePerfLogger = new PerfLogger(logger.subLogger("getAllianceCache"));
    m_getAlliancePerfLogger = new PerfLogger(logger.subLogger("getAlliance"));
    m_findSecFromStartPerfLogger = new PerfLogger(logger.subLogger("findSecFromStart"));
    m_getSamplePerfLogger = new PerfLogger(logger.subLogger("getSample"));
    m_findChassisSpeedsPerfLogger = new PerfLogger(logger.subLogger("findChassisSpeeds"));

    m_currentSample = null;

    log();
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
        "samplePose",
        new Pose2d(logSample.x, logSample.y, Rotation2d.fromRadians(logSample.heading)));
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

  public void init(
      ChassisSpeeds previousChassisSpeeds, boolean robotIsAutonomous, Pose2d currentPose) {
    double startTime = Timer.getFPGATimestamp();

    m_controller.getThetaController().reset(currentPose.getRotation().getRadians());

    m_initPerfLogger.observe(Timer.getFPGATimestamp() - startTime);
  }

  public void exit() {}

  @Override
  public synchronized ChassisSpeeds getOutput(Pose2d currentPose, Rotation2d angularVelocity) {
    double startTime = Timer.getFPGATimestamp();
    ChassisSpeeds speeds = new ChassisSpeeds();

    if (m_trajectory == null) {
      m_getOutputPerfLogger.observe(Timer.getFPGATimestamp() - startTime);
      return speeds;
    }

    double getAllianceCacheStartTime = Timer.getFPGATimestamp();
    Optional<Alliance> allianceCache = AllianceCache.Get();
    m_getAllianceCachePerfLogger.observe(Timer.getFPGATimestamp() - getAllianceCacheStartTime);

    double getAllianceStartTime = Timer.getFPGATimestamp();
    Alliance alliance = allianceCache.get();
    m_getAlliancePerfLogger.observe(Timer.getFPGATimestamp() - getAllianceStartTime);

    double findSecFromStartStartTime = Timer.getFPGATimestamp();
    double secFromStart = getTimeSecFromStart();
    m_findSecFromStartPerfLogger.observe(Timer.getFPGATimestamp() - findSecFromStartStartTime);

    double findSampleStartTime = Timer.getFPGATimestamp();
    Optional<SwerveSample> sample = m_trajectory.sampleAt(secFromStart, alliance == Alliance.Red);
    m_findSamplePerfLogger.observe(Timer.getFPGATimestamp() - findSampleStartTime);

    if (currentPose == null || sample.isEmpty()) {
      m_getOutputPerfLogger.observe(Timer.getFPGATimestamp() - startTime);
      return speeds;
    }

    double getSampleStartTime = Timer.getFPGATimestamp();
    m_currentSample = sample.get();
    m_getSamplePerfLogger.observe(Timer.getFPGATimestamp() - getSampleStartTime);

    double findChassisSpeedsStartTime = Timer.getFPGATimestamp();
    speeds =
        new ChassisSpeeds(
            m_currentSample.vx
                + m_controller.getXController().calculate(currentPose.getX(), m_currentSample.x),
            m_currentSample.vy
                + m_controller.getYController().calculate(currentPose.getY(), m_currentSample.y),
            m_currentSample.omega
                + m_controller
                    .getThetaController()
                    .calculate(currentPose.getRotation().getRadians(), m_currentSample.heading));
    m_findChassisSpeedsPerfLogger.observe(Timer.getFPGATimestamp() - findChassisSpeedsStartTime);

    m_getOutputPerfLogger.observe(Timer.getFPGATimestamp() - startTime);

    return speeds;
  }
}
