package com.team973.frc2025.subsystems;

import com.team973.frc2025.subsystems.composables.DriveWithJoysticks;
import com.team973.frc2025.subsystems.composables.DriveWithTrajectory;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public class DriveController implements Subsystem {
  private final GreyPigeon m_pigeon;
  private final Drive m_drive;

  private final Logger m_logger;

  private ControllerOption m_controllerOption = ControllerOption.DriveWithJoysticks;

  private final DriveWithJoysticks m_driveWithJoysticks;
  private final DriveWithTrajectory m_driveWithTrajectory;

  public enum RotationControl {
    OpenLoop,
    ClosedLoop,
  }

  public enum ControllerOption {
    DriveWithJoysticks,
    DriveWithTrajectory
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
        new DriveWithTrajectory(logger.subLogger("driveWithTrajectory"), m_drive);
  }

  public void setControllerOption(ControllerOption controllerOption) {
    m_controllerOption = controllerOption;
  }

  public synchronized void updateJoystickInput(double xAxis, double yAxis, double rotAxis) {
    m_driveWithJoysticks.updateJoystickInput(
        xAxis,
        yAxis,
        rotAxis,
        m_drive.getPigeon().getNormalizedYaw(),
        m_drive.getPigeon().getAngularVelocity());
  }

  public synchronized void updateTrajectory(
      Trajectory trajectory,
      double timeSeconds,
      Rotation2d rot2d,
      Rotation2d rotationalVelocityPerSecond) {
    m_driveWithTrajectory.updateTrajectory(
        trajectory, timeSeconds, rot2d, rotationalVelocityPerSecond);
  }

  public synchronized void setRotationControl(RotationControl rotationControl) {
    m_driveWithJoysticks.setRotationControl(rotationControl);
  }

  public synchronized void resetDriveWithJoysticks(Rotation2d currentYaw) {
    m_driveWithJoysticks.reset(currentYaw);
  }

  public synchronized void setHeldAngle(Rotation2d angle) {
    m_driveWithJoysticks.setHeldAngle(angle);
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

    /*
    if (m_limelight.isTargetValid()) {
      double dist = m_limelight.getHorizontalDist();
      double theta = m_limelight.getXOffset() + m_drive.getPose().getRotation().getDegrees();

      SmartDashboard.putNumber("Dist", dist);
      SmartDashboard.putNumber("Theta", theta);
      SmartDashboard.putNumber("X", dist * Math.cos(Rotation2d.fromDegrees(theta).getRadians()));
      SmartDashboard.putNumber("Y", dist * Math.sin(Rotation2d.fromDegrees(theta).getRadians()));
    }
    */
  }

  @Override
  public synchronized void syncSensors() {
    m_drive.syncSensors();
  }

  @Override
  public synchronized void update() {
    ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    switch (m_controllerOption) {
      case DriveWithJoysticks:
        currentChassisSpeeds = m_driveWithJoysticks.getOutput();
        break;
      case DriveWithTrajectory:
        currentChassisSpeeds = m_driveWithTrajectory.getOutput();
        break;
    }
    if (currentChassisSpeeds != null) {
      m_drive.setChassisSpeeds(currentChassisSpeeds);
    }
    m_drive.update();
  }

  @Override
  public synchronized void reset() {
    m_driveWithJoysticks.reset(getPigeon().getYaw());
  }
}
