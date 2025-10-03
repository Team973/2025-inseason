package com.team973.frc2025.subsystems;

import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.swerve.GreyPoseEstimator;
import com.team973.frc2025.subsystems.swerve.MegaTagSupplier;
import com.team973.frc2025.subsystems.swerve.OdometrySupplier;
import com.team973.frc2025.subsystems.swerve.SwerveModule;
import com.team973.lib.devices.GreyPigeon;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import com.team973.lib.util.SwerveModuleConfig;
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
import java.util.concurrent.atomic.AtomicReference;

public class Drive implements Subsystem {
  private RobotInfo.DriveInfo m_driveInfo;

  private static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(DriveInfo.TRACKWIDTH_METERS / 2.0, DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(DriveInfo.TRACKWIDTH_METERS / 2.0, -DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(-DriveInfo.TRACKWIDTH_METERS / 2.0, DriveInfo.WHEELBASE_METERS / 2.0),
    new Translation2d(-DriveInfo.TRACKWIDTH_METERS / 2.0, -DriveInfo.WHEELBASE_METERS / 2.0)
  };

  private final Logger m_logger;

  private final SwerveModule[] m_swerveModules;
  private ChassisSpeeds m_currentChassisSpeeds;

  private AtomicReference<Pose2d> m_estimatedPose = new AtomicReference<>(new Pose2d());
  private AtomicReference<Translation2d> m_estimatedVelocity =
      new AtomicReference<>(new Translation2d());

  private GreyPoseEstimator m_poseEstimator;
  private GreyPoseEstimator m_fusedEstimator;
  private GreyPoseEstimator m_leftOnlyEstimator;
  private GreyPoseEstimator m_rightOnlyEstimator;
  private GreyPoseEstimator m_backOnlyEstimator;

  private DriveController m_driveController;

  private final GreyPigeon m_pigeon;

  private final OdometrySupplier m_odometrySupplier;
  private final MegaTagSupplier m_leftLLSupplier;
  private final MegaTagSupplier m_rightLLSupplier;
  private final MegaTagSupplier m_backLLSupplier;

  public Drive(GreyPigeon pigeon, DriveController driveController, Logger logger) {
    m_pigeon = pigeon;
    m_driveController = driveController;
    m_logger = logger;
    m_driveInfo = RobotConfig.get().DRIVE_INFO;

    System.out.println("Front_left_constants" + m_driveInfo.FRONT_LEFT_CONSTANTS);
    m_swerveModules =
        new SwerveModule[] {
          new SwerveModule(
              0,
              new SwerveModuleConfig(
                  m_driveInfo.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                  m_driveInfo.FRONT_LEFT_MODULE_STEER_MOTOR,
                  m_driveInfo.FRONT_LEFT_MODULE_STEER_ENCODER,
                  m_driveInfo.FRONT_LEFT_MODULE_STEER_OFFSET),
              logger.subLogger("swerve/mod0")),
          new SwerveModule(
              1,
              new SwerveModuleConfig(
                  m_driveInfo.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                  m_driveInfo.FRONT_RIGHT_MODULE_STEER_MOTOR,
                  m_driveInfo.FRONT_RIGHT_MODULE_STEER_ENCODER,
                  m_driveInfo.FRONT_RIGHT_MODULE_STEER_OFFSET),
              logger.subLogger("swerve/mod1")),
          new SwerveModule(
              2,
              new SwerveModuleConfig(
                  m_driveInfo.BACK_LEFT_MODULE_DRIVE_MOTOR,
                  m_driveInfo.BACK_LEFT_MODULE_STEER_MOTOR,
                  m_driveInfo.BACK_LEFT_MODULE_STEER_ENCODER,
                  m_driveInfo.BACK_LEFT_MODULE_STEER_OFFSET),
              logger.subLogger("swerve/mod2")),
          new SwerveModule(
              3,
              new SwerveModuleConfig(
                  m_driveInfo.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                  m_driveInfo.BACK_RIGHT_MODULE_STEER_MOTOR,
                  m_driveInfo.BACK_RIGHT_MODULE_STEER_ENCODER,
                  m_driveInfo.BACK_RIGHT_MODULE_STEER_OFFSET),
              logger.subLogger("swerve/mod3"))
        };
    m_odometrySupplier =
        new OdometrySupplier(m_pigeon, m_swerveModules, logger.subLogger("providers/odometry"));

    m_currentChassisSpeeds = new ChassisSpeeds();

    m_poseEstimator =
        new GreyPoseEstimator(
            m_pigeon, m_driveController, m_odometrySupplier, logger.subLogger("estimators/main"));

    m_leftLLSupplier =
        new MegaTagSupplier(
            "limelight-left",
            m_pigeon,
            // These measurements were derived from CAD.
            new Pose3d(
                new Translation3d(0.108, -0.275, 0.908),
                new Rotation3d(
                    Rotation2d.fromDegrees(-90.0).getRadians(),
                    Rotation2d.fromDegrees(-30.0).getRadians(),
                    Rotation2d.fromDegrees(-30.0).getRadians())),
            logger.subLogger("providers/ll-left"));
    m_rightLLSupplier =
        new MegaTagSupplier(
            "limelight-right",
            m_pigeon,
            new Pose3d(
                new Translation3d(0.108, 0.275, 0.908),
                new Rotation3d(
                    Rotation2d.fromDegrees(90.0).getRadians(),
                    Rotation2d.fromDegrees(-30.0).getRadians(),
                    Rotation2d.fromDegrees(30.0).getRadians())),
            logger.subLogger("providers/ll-right"));
    m_backLLSupplier =
        new MegaTagSupplier(
            "limelight-back",
            m_pigeon,
            new Pose3d(
                new Translation3d(0.0, 0.108, 0.968), new Rotation3d(Rotation2d.fromDegrees(180))),
            logger.subLogger("providers/ll-back"));

    m_fusedEstimator =
        new GreyPoseEstimator(
            m_pigeon, m_driveController, m_odometrySupplier, logger.subLogger("estimators/fused"));
    m_leftLLSupplier.addReceiver(m_fusedEstimator);
    m_rightLLSupplier.addReceiver(m_fusedEstimator);
    m_backLLSupplier.addReceiver(m_fusedEstimator);

    m_leftOnlyEstimator =
        new GreyPoseEstimator(
            m_pigeon,
            m_driveController,
            m_odometrySupplier,
            logger.subLogger("estimators/leftOnly"));
    m_leftLLSupplier.addReceiver(m_leftOnlyEstimator);
    m_rightOnlyEstimator =
        new GreyPoseEstimator(
            m_pigeon,
            driveController,
            m_odometrySupplier,
            logger.subLogger("estimators/rightOnly"));
    m_rightLLSupplier.addReceiver(m_rightOnlyEstimator);
    m_backOnlyEstimator =
        new GreyPoseEstimator(
            m_pigeon, driveController, m_odometrySupplier, logger.subLogger("estimators/backOnly"));
    m_backLLSupplier.addReceiver(m_backOnlyEstimator);
  }

  public void startOdometrey() {
    m_odometrySupplier.start();
  }

  public GreyPigeon getPigeon() {
    return m_pigeon;
  }

  public GreyPoseEstimator getPoseEstimator() {
    return m_fusedEstimator;
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
        desiredStates, m_driveInfo.MAX_VELOCITY_METERS_PER_SECOND);

    double states[] = new double[8];
    int index = 0;
    for (SwerveModule mod : m_swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
      states[index] = desiredStates[mod.moduleNumber].angle.getDegrees();
      states[index + 1] = desiredStates[mod.moduleNumber].speedMetersPerSecond;
      index += 2;
    }

    // SmartDashboard.putNumberArray("swerve/setpoints", states);
  }

  public Pose2d getPose() {
    return m_estimatedPose.get();
  }

  public Translation2d getVelocity() {
    return m_estimatedVelocity.get();
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
    m_leftOnlyEstimator.resetPosition(pose);
    m_rightOnlyEstimator.resetPosition(pose);
    m_backOnlyEstimator.resetPosition(pose);
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
    m_logger.log("X Meters Per Second", m_poseEstimator.getVelocityMetersPerSeconds().getX());
    m_logger.log("Y Meters Per Second", m_poseEstimator.getVelocityMetersPerSeconds().getY());

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
    m_rightOnlyEstimator.log();
    m_leftOnlyEstimator.log();
    m_backOnlyEstimator.log();
    m_leftLLSupplier.log();
    m_rightLLSupplier.log();
    m_backLLSupplier.log();
    m_odometrySupplier.log();
  }

  @Override
  public void syncSensors() {
    m_leftLLSupplier.syncSensors();
    m_rightLLSupplier.syncSensors();
    m_backLLSupplier.syncSensors();
  }

  public synchronized void syncSensorsHighFreq() {
    // Use the fused estimator for our position but the odometry-only estimator
    // for our velocity.
    m_estimatedPose.set(m_fusedEstimator.getPoseMeters());
    m_estimatedVelocity.set(m_poseEstimator.getVelocityMetersPerSeconds());
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
        m_driveInfo.getSwerveDriveKinmatics().toSwerveModuleStates(updated_chassis_speeds);

    setModuleStates(swerveModuleStates);
  }

  public void reset() {
    m_pigeon.reset();
    resetOdometry(new Pose2d());
  }
}
