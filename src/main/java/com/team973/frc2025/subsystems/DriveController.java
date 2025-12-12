package com.team973.frc2025.subsystems;

import com.team973.frc2025.subsystems.composables.DriveWithJoysticks;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight;
import com.team973.frc2025.subsystems.composables.DriveWithTrajectory;
import com.team973.frc2025.subsystems.swerve.SwerveModuleIO;
import com.team973.lib.devices.GreyPigeonIO;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.concurrent.atomic.AtomicBoolean;

public class DriveController implements Subsystem {
  private final Drive m_drive;

  private final Logger m_logger;

  private ControllerOption m_controllerOption = ControllerOption.DriveWithJoysticks;

  private final DriveWithJoysticks m_driveWithJoysticks;
  private final DriveWithTrajectory m_driveWithTrajectory;
  private final DriveWithLimelight m_driveWithLimelight;

  private ChassisSpeeds m_currentChassisSpeeds;

  private boolean m_robotIsAutonomous = true;

  public enum RotationControl {
    OpenLoop,
    ClosedLoop,
  }

  public enum ControllerOption {
    DriveWithJoysticks,
    DriveWithTrajectory,
    DriveWithLimelight,
  }

  public static class AnglePresets {
    public static final Rotation2d TOWARDS_SPEAKER = Rotation2d.fromDegrees(180);
    public static final Rotation2d SOURCE_RED = Rotation2d.fromDegrees(180 + 30);
    public static final Rotation2d SOURCE_BLUE = Rotation2d.fromDegrees(180 - 30);
  }

  public void startOdometrey() {
    m_drive.startOdometrey();
  }

  public DriveController(
      Logger logger,
      AtomicBoolean readyToScore,
      AtomicBoolean readyToBackOff,
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight,
      GreyPigeonIO pigeon) {
    m_drive = new Drive(pigeon, this, frontLeft, frontRight, backLeft, backRight, logger);

    m_logger = logger.subLogger("controller");

    m_driveWithJoysticks = new DriveWithJoysticks();
    m_driveWithTrajectory =
        new DriveWithTrajectory(m_logger.subLogger("driveWithTrajectory"), m_drive);

    m_driveWithLimelight =
        new DriveWithLimelight(
            m_logger.subLogger("driveWithLimelight"), readyToScore, readyToBackOff);

    m_currentChassisSpeeds = new ChassisSpeeds();
  }

  public synchronized void setControllerOption(ControllerOption controllerOption) {
    if (controllerOption != m_controllerOption) {
      m_driveWithJoysticks.reset(m_drive.getPoseEstimator().getPoseMeters().getRotation());

      getComposableFromControllerOption(m_controllerOption).exit();

      getComposableFromControllerOption(controllerOption)
          .init(m_currentChassisSpeeds, m_robotIsAutonomous, getPose());

      m_controllerOption = controllerOption;
    }
  }

  public DriveWithJoysticks getDriveWithJoysticks() {
    return m_driveWithJoysticks;
  }

  public DriveWithTrajectory getDriveWithTrajectory() {
    return m_driveWithTrajectory;
  }

  public DriveWithLimelight getDriveWithLimelight() {
    return m_driveWithLimelight;
  }

  public boolean isNearApproach() {
    return getDriveWithLimelight().isNearApproach(getPose());
  }

  public synchronized GreyPigeonIO getPigeon() {
    return m_drive.getPigeon();
  }

  public synchronized Pose2d getPose() {
    return m_drive.getPose();
  }

  public synchronized Translation2d getVelocity() {
    return m_drive.getVelocity();
  }

  public synchronized void resetOdometry(Pose2d pose2d) {
    m_drive.resetOdometry(pose2d);
    m_driveWithJoysticks.reset(pose2d.getRotation());
  }

  public synchronized void resetAngle(Rotation2d angle) {
    m_drive.resetOdometry(new Pose2d(getPose().getTranslation(), angle));
    m_driveWithJoysticks.reset(angle);
  }

  @Override
  public void log() {
    m_drive.log();
    m_driveWithTrajectory.log();
    m_driveWithLimelight.log();

    m_logger.log("chassis speeds/vx", m_currentChassisSpeeds.vxMetersPerSecond);
    m_logger.log("chassis speeds/vy", m_currentChassisSpeeds.vyMetersPerSecond);
    m_logger.log("chassis speeds/omega RadPS", m_currentChassisSpeeds.omegaRadiansPerSecond);
    m_logger.log("controller", m_controllerOption.toString());
  }

  private DriveComposable getComposableFromControllerOption(ControllerOption option) {
    switch (option) {
      case DriveWithJoysticks:
        return m_driveWithJoysticks;
      case DriveWithTrajectory:
        return m_driveWithTrajectory;
      case DriveWithLimelight:
        return m_driveWithLimelight;
      default:
        return m_driveWithJoysticks;
    }
  }

  public void setRobotIsAutonomous(boolean robotIsAutonomous) {
    m_robotIsAutonomous = robotIsAutonomous;
  }

  public void syncSensors() {
    if (m_drive == null) {
      return;
    }
    m_drive.syncSensors();
  }

  public synchronized void syncSensorsHighFreq() {
    if (m_drive == null) {
      // TODO: Need to re-think initialization here.
      return;
    }
    m_drive.syncSensorsHighFreq();
  }

  @Override
  public synchronized void update() {
    ChassisSpeeds currentChassisSpeeds =
        getComposableFromControllerOption(m_controllerOption)
            .getOutput(getPose(), m_drive.getPigeon().getAngularVelocity());

    if (currentChassisSpeeds != null) {
      m_drive.setChassisSpeeds(currentChassisSpeeds);
      m_currentChassisSpeeds = currentChassisSpeeds;
    }

    m_drive.update();
  }

  @Override
  public synchronized void reset() {
    m_driveWithJoysticks.reset(getPigeon().getYaw());
  }
}
