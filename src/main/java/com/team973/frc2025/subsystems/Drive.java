package com.team973.frc2025.subsystems;

import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.swerve.GreyPoseEstimator;
import com.team973.frc2025.subsystems.swerve.MegaTagSupplier;
import com.team973.frc2025.subsystems.swerve.OdometrySupplier;
import com.team973.frc2025.subsystems.swerve.SwerveModule;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive implements Subsystem {
  private static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(DriveInfo.TRACKWIDTH_METERS / 2.0, DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(DriveInfo.TRACKWIDTH_METERS / 2.0, -DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(-DriveInfo.TRACKWIDTH_METERS / 2.0, DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(-DriveInfo.TRACKWIDTH_METERS / 2.0, -DriveInfo.WHEELBASE_METERS / 2.0)
  };

  private final Logger m_logger;

  private final SwerveModule[] m_swerveModules;
  private ChassisSpeeds m_currentChassisSpeeds;
  private Pose2d m_estimatedPose = new Pose2d();
  private Translation2d m_estimatedVelocity = new Translation2d();
  private GreyPoseEstimator m_poseEstimator;

  private GreyPoseEstimator m_fusedEstimator;
  private DriveController m_driveController;

  private final GreyPigeon m_pigeon;

  private Rotation2d m_targetRobotAngle = new Rotation2d();
  private final OdometrySupplier m_odometrySupplier;
  private final MegaTagSupplier m_backLLSupplier;
  private final MegaTagSupplier m_frontLLSupplier;

  public Drive(GreyPigeon pigeon, DriveController driveController, Logger logger) {
    m_pigeon = pigeon;
    m_driveController = driveController;
    m_logger = logger;
    m_swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, DriveInfo.FRONT_LEFT_CONSTANTS, logger.subLogger("swerve/mod0")),
          new SwerveModule(1, DriveInfo.FRONT_RIGHT_CONSTANTS, logger.subLogger("swerve/mod1")),
          new SwerveModule(2, DriveInfo.BACK_LEFT_CONSTANTS, logger.subLogger("swerve/mod2")),
          new SwerveModule(3, DriveInfo.BACK_RIGHT_CONSTANTS, logger.subLogger("swerve/mod3"))
        };
    m_odometrySupplier =
        new OdometrySupplier(m_pigeon, m_swerveModules, logger.subLogger("providers/odometry"));

    m_currentChassisSpeeds = new ChassisSpeeds();

    m_poseEstimator =
        new GreyPoseEstimator(
            m_pigeon, m_driveController, m_odometrySupplier, logger.subLogger("estimators/main"));

    m_backLLSupplier =
        new MegaTagSupplier(
            "limelight-back",
            m_pigeon,
            // TODO: Waiting on these measurements
            new Pose3d(
                new Translation3d(-0.3683, 0.0, 1.1811),
                new Rotation3d(
                    0.0,
                    Rotation2d.fromDegrees(30.5).getRadians(),
                    Rotation2d.fromDegrees(180).getRadians())),
            logger.subLogger("providers/ll-back"));
    m_frontLLSupplier =
        new MegaTagSupplier(
            "limelight-front",
            m_pigeon,
            new Pose3d(new Translation3d(0.324, 0.0, 0.178), new Rotation3d(0.0, 0.0, 0.0)),
            logger.subLogger("providers/ll-front"));

    m_fusedEstimator =
        new GreyPoseEstimator(
            m_pigeon, m_driveController, m_odometrySupplier, logger.subLogger("estimators/fused"));
    m_backLLSupplier.addReceiver(m_fusedEstimator);
    m_frontLLSupplier.addReceiver(m_fusedEstimator);
  }

  public void startOdometrey() {
    m_odometrySupplier.start();
  }

  public GreyPigeon getPigeon() {
    return m_pigeon;
  }

  public GreyPoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_currentChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, m_pigeon.getYaw());
  }

  public void xOutModules() {
    // Side effect: any drive input that goes above the anti-jitter threshold
    // overrides this
    int index = 0;
    for (SwerveModule mod : m_swerveModules) {
      double angleToCenter =
          Math.atan2(MODULE_LOCATIONS[index].getY(), MODULE_LOCATIONS[index].getX());
      index++;

      mod.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromRadians(angleToCenter)), true);
    }
  }

  /* Used by Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveInfo.MAX_VELOCITY_METERS_PER_SECOND);

    double states[] = new double[8];
    int index = 0;
    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
      states[index] = desiredStates[mod.moduleNumber].angle.getDegrees();
      states[index + 1] = desiredStates[mod.moduleNumber].speedMetersPerSecond;
      index += 2;
    }

    SmartDashboard.putNumberArray("swerve/setpoints", states);
  }

  public Pose2d getPose() {
    return m_estimatedPose;
  }

  public Translation2d getVelocity() {
    return m_estimatedVelocity;
  }

  public static boolean comparePoses(
      Pose2d poseA, Pose2d poseB, double distanceTolerance, double angleTolerance) {
    return poseA.getTranslation().getDistance(poseB.getTranslation()) < distanceTolerance
        && poseA.getRotation().minus(poseB.getRotation()).getDegrees() < angleTolerance;
  }

  public void resetOdometry(Pose2d pose) {
    m_pigeon.setYawOffset(m_pigeon.getRawYaw().minus(pose.getRotation()));
    m_poseEstimator.resetPosition(pose);
    m_fusedEstimator.resetPosition(pose);
    syncSensors();
  }

  public void resetModules() {
    for (SwerveModule mod : m_swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public void enableBrakeMode() {
    for (var mod : m_swerveModules) {
      mod.driveBrake();
    }
  }

  public void disableBrakeMode() {
    for (var mod : m_swerveModules) {
      mod.driveNeutral();
    }
  }

  public void log() {

    double states[] = new double[8];
    int index = 0;

    for (SwerveModule mod : m_swerveModules) {
      mod.log();
      states[index] = mod.getState().angle.getDegrees();
      states[index + 1] = mod.getState().speedMetersPerSecond;
      index += 2;
    }

    m_logger.log("Actual", states);
    m_pigeon.log();
    m_poseEstimator.log();
    m_fusedEstimator.log();
    m_backLLSupplier.log();
    m_frontLLSupplier.log();
    m_odometrySupplier.log();
  }

  @Override
  public void syncSensors() {
    m_backLLSupplier.syncSensors();
    m_frontLLSupplier.syncSensors();
    m_estimatedPose = m_poseEstimator.getPoseMeters();
    m_estimatedVelocity = m_poseEstimator.getVelocityMetersPerSeconds();
  }

  @Override
  public void update() {
    Pose2d robot_pose_vel =
        new Pose2d(
            m_currentChassisSpeeds.vxMetersPerSecond * 0.03,
            m_currentChassisSpeeds.vyMetersPerSecond * 0.03,
            new Rotation2d(m_currentChassisSpeeds.omegaRadiansPerSecond * 0.03));
    Pose2d robot_cur_pose = new Pose2d();
    Twist2d twist_vel = robot_cur_pose.log(robot_pose_vel);
    ChassisSpeeds updated_chassis_speeds =
        new ChassisSpeeds(twist_vel.dx / 0.03, twist_vel.dy / 0.03, twist_vel.dtheta / 0.03);

    SwerveModuleState[] swerveModuleStates =
        DriveInfo.SWERVE_KINEMATICS.toSwerveModuleStates(updated_chassis_speeds);

    setModuleStates(swerveModuleStates);

    SmartDashboard.putNumber("Yaw", m_pigeon.getYaw().getDegrees());
  }

  public void reset() {
    m_pigeon.reset();
    resetOdometry(new Pose2d());
  }
}
