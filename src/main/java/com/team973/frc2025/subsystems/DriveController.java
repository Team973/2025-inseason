package com.team973.frc2025.subsystems;

import com.team973.frc2025.subsystems.composables.DriveWithJoysticks;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight;
import com.team973.frc2025.subsystems.composables.DriveWithTrajectory;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveController implements Subsystem {
  private final GreyPigeon m_pigeon;
  private final Drive m_drive;

  private final Logger m_logger;

  private ControllerOption m_controllerOption = ControllerOption.DriveWithJoysticks;

  private final DriveWithJoysticks m_driveWithJoysticks;
  private final DriveWithTrajectory m_driveWithTrajectory;
  private final DriveWithLimelight m_driveWithLimelight;

  private ChassisSpeeds m_currentChassisSpeeds;

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

  public DriveController(Logger logger) {
    m_pigeon = new GreyPigeon(logger.subLogger("pigeon"));
    m_drive = new Drive(m_pigeon, this, logger);

    m_logger = logger.subLogger("controller");

    m_driveWithJoysticks = new DriveWithJoysticks();
    m_driveWithTrajectory =
        new DriveWithTrajectory(m_logger.subLogger("driveWithTrajectory"), m_drive);

    m_driveWithLimelight =
        new DriveWithLimelight(
            m_drive.getPoseEstimator(), m_logger.subLogger("driveWithLimelight"));

    m_currentChassisSpeeds = new ChassisSpeeds();
  }

  public void setControllerOption(ControllerOption controllerOption) {
    if (controllerOption != m_controllerOption) {
      m_controllerOption = controllerOption;
      m_driveWithJoysticks.reset(m_drive.getPoseEstimator().getPoseMeters().getRotation());
      getComposableFromControllerOption(controllerOption).init();
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

  public synchronized GreyPigeon getPigeon() {
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
    switch (m_controllerOption) {
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

  @Override
  public synchronized void syncSensors() {
    m_drive.syncSensors();
    m_driveWithTrajectory.updatePose(getPose());
    m_driveWithJoysticks.updateAngle(
        m_drive.getPigeon().getNormalizedYaw(), m_drive.getPigeon().getAngularVelocity());
  }

  @Override
  public synchronized void update() {
    ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    DriveComposable currentComposable = getComposableFromControllerOption(m_controllerOption);

    currentChassisSpeeds = currentComposable.getOutput();

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
