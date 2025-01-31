package com.team973.frc2025.subsystems.composables;

import com.team973.frc2025.subsystems.Drive;
import com.team973.frc2025.subsystems.swerve.GreyPoseEstimator;
import com.team973.lib.util.AprilTag;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.Logger;
import com.team973.lib.util.TargetPositionRelativeToAprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class DriveWithLimelight extends DriveComposable {
  private final GreyPoseEstimator m_poseEstimator;

  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private final Logger m_logger;

  private TargetPositionRelativeToAprilTag m_target = null;

  private Pose2d m_targetInitialPose = new Pose2d();
  private Pose2d m_targetFinalPose = new Pose2d();

  private TargetMode m_targetMode = TargetMode.Initial;

  private enum TargetMode {
    Initial,
    Final
  }

  public static class TargetPositions {
    private static final Translation2d LEFT_REEF_INITIAL_TARGET = new Translation2d(0.0, 0.6);
    private static final Translation2d RIGHT_REEF_INITIAL_TARGET = new Translation2d(0.0, 0.6);
    private static final double REEF_FINAL_DIST = 0.1;

    private static final Translation2d HP_INITIAL_TARGET = new Translation2d(0.0, 0.5);

    public static final TargetPositionRelativeToAprilTag TEST_ONE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(0), new Translation2d(0.0, 0.2), 0, Rotation2d.fromDegrees(180));
    public static final TargetPositionRelativeToAprilTag TEST_TWO =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(-1), new Translation2d(0.0, 0.3), 0.1, new Rotation2d());

    public static final TargetPositionRelativeToAprilTag HPL =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(1), HP_INITIAL_TARGET, 0.0, Rotation2d.fromDegrees(180));
    public static final TargetPositionRelativeToAprilTag HPR =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(2), HP_INITIAL_TARGET, 0.0, Rotation2d.fromDegrees(180));
    public static final TargetPositionRelativeToAprilTag ONE_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), LEFT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag ONE_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), RIGHT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag TWO_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), LEFT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag TWO_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), RIGHT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag THREE_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), LEFT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag THREE_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), RIGHT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag FOUR_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), LEFT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag FOUR_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), RIGHT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag FIVE_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), LEFT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag FIVE_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), RIGHT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag SIX_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), LEFT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag SIX_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), RIGHT_REEF_INITIAL_TARGET, REEF_FINAL_DIST, new Rotation2d());
  }

  public DriveWithLimelight(GreyPoseEstimator poseEstimator, Logger logger) {
    m_poseEstimator = poseEstimator;

    m_xController =
        new ProfiledPIDController(12.0, 0, 0, new TrapezoidProfile.Constraints(0.2, 0.1));
    m_yController =
        new ProfiledPIDController(12.0, 0, 0, new TrapezoidProfile.Constraints(0.2, 0.1));
    m_thetaController =
        new ProfiledPIDController(8.0, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));

    m_thetaController.enableContinuousInput(
        Units.degreesToRadians(-180.0), Units.degreesToRadians(180.0));

    m_logger = logger;
  }

  public void setTargetPosition(TargetPositionRelativeToAprilTag target) {
    if (m_target != target) {
      Pose3d aprilTagLocation = target.getAprilTagPose();

      m_targetInitialPose =
          new Pose2d(
              aprilTagLocation
                  .getTranslation()
                  .toTranslation2d()
                  .plus(
                      new Translation2d(
                          target.getInitialTarget().getX(),
                          aprilTagLocation
                              .getRotation()
                              .toRotation2d()
                              .plus(Rotation2d.fromDegrees(90.0))))
                  .plus(
                      new Translation2d(
                          target.getInitialTarget().getY(),
                          aprilTagLocation
                              .getRotation()
                              .toRotation2d()
                              .plus(Rotation2d.fromDegrees(180)))),
              aprilTagLocation.getRotation().toRotation2d().plus(target.getTargetAngle()));

      m_targetFinalPose =
          m_targetInitialPose.plus(
              new Transform2d(
                  new Translation2d(
                      target.getFinalDist(),
                      aprilTagLocation
                          .getRotation()
                          .toRotation2d()
                          .plus(Rotation2d.fromDegrees(180))),
                  new Rotation2d()));

      m_targetMode = TargetMode.Initial;
      m_target = target;
    }
  }

  public void log() {
    m_logger.log("Target Mode", m_targetMode.toString());

    m_logger.log("Target Initial Pose/x", m_targetInitialPose.getX());
    m_logger.log("Target Initial Pose/y", m_targetInitialPose.getY());
    m_logger.log("Target Initial Pose/degrees", m_targetInitialPose.getRotation().getDegrees());

    m_logger.log("Target Final Pose/x", m_targetFinalPose.getX());
    m_logger.log("Target Final Pose/y", m_targetFinalPose.getY());
    m_logger.log("Target Final Pose/degrees", m_targetFinalPose.getRotation().getDegrees());

    m_logger.log("X Controller Target Position", m_xController.getSetpoint().position);
    m_logger.log("Y Controller Target Position", m_yController.getSetpoint().position);
    m_logger.log(
        "Theta Controller Target Position Deg",
        Math.toDegrees(m_thetaController.getSetpoint().position));
  }

  public void init() {
    m_xController.reset(m_poseEstimator.getPoseMeters().getX());
    m_yController.reset(m_poseEstimator.getPoseMeters().getY());
    m_thetaController.reset(m_poseEstimator.getPoseMeters().getRotation().getRadians());
  }

  public ChassisSpeeds getOutput() {
    if (m_target == null) {
      return new ChassisSpeeds(0, 0, 0);
    }

    if (Drive.comparePoses(m_poseEstimator.getPoseMeters(), m_targetInitialPose, 0.03, 5)) {
      m_targetMode = TargetMode.Final;
    }

    switch (m_targetMode) {
      case Initial:
        return new ChassisSpeeds(
            m_xController.calculate(
                m_poseEstimator.getPoseMeters().getX(), m_targetInitialPose.getX()),
            m_yController.calculate(
                m_poseEstimator.getPoseMeters().getY(), m_targetInitialPose.getY()),
            m_thetaController.calculate(
                m_poseEstimator.getPoseMeters().getRotation().getRadians(),
                m_targetInitialPose.getRotation().getRadians()));
      case Final:
        return new ChassisSpeeds(
            m_xController.calculate(
                m_poseEstimator.getPoseMeters().getX(), m_targetFinalPose.getX()),
            m_yController.calculate(
                m_poseEstimator.getPoseMeters().getY(), m_targetFinalPose.getY()),
            m_thetaController.calculate(
                m_poseEstimator.getPoseMeters().getRotation().getRadians(),
                m_targetFinalPose.getRotation().getRadians()));
      default:
        throw new IllegalArgumentException(m_targetMode.toString());
    }
  }
}
