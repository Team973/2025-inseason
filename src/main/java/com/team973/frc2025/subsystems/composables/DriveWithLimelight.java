package com.team973.frc2025.subsystems.composables;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Drive;
import com.team973.frc2025.subsystems.swerve.GreyPoseEstimator;
import com.team973.lib.util.AprilTag;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.Logger;
import com.team973.lib.util.TargetPositionRelativeToAprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

public class DriveWithLimelight extends DriveComposable {
  private static final double TARGET_DISTANCE_TOLERANCE_METERS = 0.05;
  private static final double TARGET_ANGLE_TOLERANCE_DEG = 5.0;

  private final GreyPoseEstimator m_poseEstimator;

  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private final Logger m_logger;

  private TargetPositionRelativeToAprilTag m_target = null;

  private Pose2d m_approachPose = new Pose2d();
  private Pose2d m_scoringPose = new Pose2d();

  private Pose2d m_approachPoseLog = new Pose2d();
  private Pose2d m_scoringPoseLog = new Pose2d();

  private ReefFace m_targetReefFace = ReefFace.A;
  private ReefSide m_targetReefSide = ReefSide.Left;

  private BooleanSupplier m_targetScoringPoseGate = () -> true;
  private BooleanSupplier m_targetBackOffPoseGate = () -> false;

  private TargetStage m_targetStage = TargetStage.MoveToApproach;

  public enum ReefSide {
    Left,
    Right
  }

  public enum ReefFace {
    A,
    B,
    C,
    D,
    E,
    F
  }

  public enum TargetStage {
    MoveToApproach,
    Approach,
    MoveToScoring,
    Scoring,
    MoveToBackOff,
    BackOff
  }

  public static class TargetPositions {
    private static final double REEF_WIDTH_METERS = 0.33;
    private static final Translation2d LEFT_REEF_APPROACH_TARGET =
        new Translation2d(-REEF_WIDTH_METERS / 2.0, 0.8);
    private static final Translation2d RIGHT_REEF_APPROACH_TARGET =
        new Translation2d(REEF_WIDTH_METERS / 2.0, 0.8);
    private static final double REEF_SCORING_DIST = 0.45;

    private static final Translation2d HP_APPROACH_TARGET = new Translation2d(0.0, 0.5);

    public static final TargetPositionRelativeToAprilTag TEST_ONE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(0), new Translation2d(0.1, 0.2), 0, Rotation2d.fromDegrees(180));
    public static final TargetPositionRelativeToAprilTag TEST_TWO =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(-1), new Translation2d(0.1, 0.3), 0.1, new Rotation2d());

    public static final TargetPositionRelativeToAprilTag HPL =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(1), HP_APPROACH_TARGET, 0.0, Rotation2d.fromDegrees(180));
    public static final TargetPositionRelativeToAprilTag HPR =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(2), HP_APPROACH_TARGET, 0.0, Rotation2d.fromDegrees(180));
    public static final TargetPositionRelativeToAprilTag A_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag A_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag B_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag B_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag C_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag C_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag D_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag D_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag E_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag E_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag F_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag F_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
  }

  public DriveWithLimelight(GreyPoseEstimator poseEstimator, Logger logger) {
    m_poseEstimator = poseEstimator;

    double controlPeriodSeconds = 1.0 / RobotInfo.DriveInfo.STATUS_SIGNAL_FREQUENCY;
    m_xController =
        new ProfiledPIDController(
            4.0, 0, 0, new TrapezoidProfile.Constraints(0.8, 0.2), controlPeriodSeconds);
    m_yController =
        new ProfiledPIDController(
            4.0, 0, 0, new TrapezoidProfile.Constraints(0.8, 0.2), controlPeriodSeconds);
    m_thetaController =
        new ProfiledPIDController(
            8.0, 0, 0, new TrapezoidProfile.Constraints(1.0, 0.6), controlPeriodSeconds);

    m_thetaController.enableContinuousInput(
        Units.degreesToRadians(0.0), Units.degreesToRadians(360.0));

    m_logger = logger;
  }

  public void setTargetSide(ReefSide side) {
    m_targetReefSide = side;

    m_approachPoseLog = getTargetReefPosition().getApproachPose();
    m_scoringPoseLog = getTargetReefPosition().getScoringPose();
  }

  public void setTargetReefFace(ReefFace face) {
    m_targetReefFace = face;

    m_approachPoseLog = getTargetReefPosition().getApproachPose();
    m_scoringPoseLog = getTargetReefPosition().getScoringPose();
  }

  public TargetPositionRelativeToAprilTag getTargetReefPosition() {
    switch (m_targetReefFace) {
      case A:
        return m_targetReefSide == ReefSide.Left ? TargetPositions.A_L : TargetPositions.A_R;
      case B:
        return m_targetReefSide == ReefSide.Left ? TargetPositions.B_L : TargetPositions.B_R;
      case C:
        return m_targetReefSide == ReefSide.Left ? TargetPositions.C_L : TargetPositions.C_R;
      case D:
        return m_targetReefSide == ReefSide.Left ? TargetPositions.D_L : TargetPositions.D_R;
      case E:
        return m_targetReefSide == ReefSide.Left ? TargetPositions.E_L : TargetPositions.E_R;
      case F:
        return m_targetReefSide == ReefSide.Left ? TargetPositions.F_L : TargetPositions.F_R;
      default:
        throw new IllegalArgumentException("Invalid reef face: " + m_targetReefFace);
    }
  }

  public TargetStage getTargetStage() {
    return m_targetStage;
  }

  public void targetReefPosition(
      BooleanSupplier targetScoringPoseGate, BooleanSupplier targetBackOffPoseGate) {
    if (getTargetReefPosition() != m_target) {
      m_approachPose = getTargetReefPosition().getApproachPose();
      m_scoringPose = getTargetReefPosition().getScoringPose();

      m_approachPoseLog = getTargetReefPosition().getApproachPose();
      m_scoringPoseLog = getTargetReefPosition().getScoringPose();

      m_target = getTargetReefPosition();

      m_targetStage = TargetStage.MoveToApproach;
    }

    m_targetScoringPoseGate = targetScoringPoseGate;
    m_targetBackOffPoseGate = targetBackOffPoseGate;
  }

  public void log() {
    SmartDashboard.putString("DB/String 6", "Reef Face: " + m_targetReefFace);
    SmartDashboard.putString("DB/String 7", "Reef Side: " + m_targetReefSide);

    m_logger.log("Target Side", m_targetReefSide.toString());
    m_logger.log("Target Stage", m_targetStage.toString());

    m_logger.log("Target Scoring Pose Gate", m_targetScoringPoseGate.getAsBoolean());
    m_logger.log("Target BackOff Pose Gate", m_targetBackOffPoseGate.getAsBoolean());

    m_logger.log("Is At Approach", isAtApproach());
    m_logger.log("Is At Scoring", isAtScoring());

    m_logger.log(
        "Approach Error/X", m_poseEstimator.getPoseMeters().getX() - m_approachPose.getX());
    m_logger.log(
        "Approach Error/Y", m_poseEstimator.getPoseMeters().getY() - m_approachPose.getY());
    m_logger.log(
        "Approach Error/Deg",
        m_poseEstimator.getPoseMeters().getRotation().getDegrees()
            - m_approachPose.getRotation().getDegrees());

    m_logger.log("Scoring Error/X", m_poseEstimator.getPoseMeters().getX() - m_scoringPose.getX());
    m_logger.log("Scoring Error/Y", m_poseEstimator.getPoseMeters().getY() - m_scoringPose.getY());
    m_logger.log(
        "Scoring Error/Deg",
        m_poseEstimator.getPoseMeters().getRotation().getDegrees()
            - m_scoringPose.getRotation().getDegrees());

    m_logger.log(
        "Target Initial Pose",
        new double[] {
          m_approachPose.getX(), m_approachPose.getY(), m_approachPose.getRotation().getRadians()
        });

    m_logger.log(
        "Target Initial Pose Log",
        new double[] {
          m_approachPoseLog.getX(),
          m_approachPoseLog.getY(),
          m_approachPoseLog.getRotation().getRadians()
        });

    m_logger.log(
        "Target Final Pose",
        new double[] {
          m_scoringPose.getX(), m_scoringPose.getY(), m_scoringPose.getRotation().getRadians()
        });

    m_logger.log(
        "Controller Target Position",
        new double[] {
          m_xController.getSetpoint().position,
          m_yController.getSetpoint().position,
          m_thetaController.getSetpoint().position
        });
    m_logger.log("x-state/velocity", m_xController.getSetpoint().velocity);
    m_logger.log("x-state/position", m_xController.getSetpoint().position);
    m_logger.log("y-state/velocity", m_yController.getSetpoint().velocity);
    m_logger.log("y-state/position", m_yController.getSetpoint().position);

    m_logger.log(
        "Target Final Pose Log",
        new double[] {
          m_scoringPoseLog.getX(),
          m_scoringPoseLog.getY(),
          m_scoringPoseLog.getRotation().getRadians()
        });
    Pose2d currentTargetPose2d = getCurrentTargetPose2d();
    m_logger.log(
        "dist to target",
        m_poseEstimator
            .getPoseMeters()
            .getTranslation()
            .getDistance(currentTargetPose2d.getTranslation()));
  }

  private boolean isAtApproach() {
    return Drive.comparePoses(
        m_poseEstimator.getPoseMeters(),
        m_approachPose,
        TARGET_DISTANCE_TOLERANCE_METERS * 1.2,
        TARGET_ANGLE_TOLERANCE_DEG * 1.2);
  }

  private boolean isAtScoring() {
    return Drive.comparePoses(
        m_poseEstimator.getPoseMeters(),
        m_scoringPose,
        TARGET_DISTANCE_TOLERANCE_METERS * 1.2,
        TARGET_ANGLE_TOLERANCE_DEG * 1.2);
  }

  public Pose2d getCurrentTargetPose2d() {
    switch (m_targetStage) {
      case MoveToBackOff:
      case BackOff:
      case MoveToApproach:
      case Approach:
        return m_approachPose;
      case MoveToScoring:
      case Scoring:
        return m_scoringPose;
      default:
        throw new IllegalArgumentException(m_targetStage.toString());
    }
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

    switch (m_targetStage) {
      case MoveToApproach:
        if (isAtApproach()) {
          m_targetStage = TargetStage.Approach;
        }
        break;
      case Approach:
        if (m_targetScoringPoseGate.getAsBoolean()) {
          m_targetStage = TargetStage.MoveToScoring;
        }
        break;
      case MoveToScoring:
        if (isAtScoring()) {
          m_targetStage = TargetStage.Scoring;
        }
        break;
      case Scoring:
        if (m_targetBackOffPoseGate.getAsBoolean()) {
          m_targetStage = TargetStage.MoveToBackOff;
        }
        break;
      case MoveToBackOff:
        if (isAtApproach()) {
          m_targetStage = TargetStage.BackOff;
        }
        break;
      case BackOff:
        break;
    }

    return new ChassisSpeeds(
        m_xController.calculate(
                m_poseEstimator.getPoseMeters().getX(), getCurrentTargetPose2d().getX())
            + m_xController.getSetpoint().velocity,
        m_yController.calculate(
                m_poseEstimator.getPoseMeters().getY(), getCurrentTargetPose2d().getY())
            + m_yController.getSetpoint().velocity,
        m_thetaController.calculate(
                m_poseEstimator.getPoseMeters().getRotation().getRadians(),
                getCurrentTargetPose2d().getRotation().getRadians())
            + m_thetaController.getSetpoint().velocity);
  }
}
