package com.team973.frc2025.subsystems.composables;

import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Drive;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class DriveWithLimelight extends DriveComposable {
  private static final double SCORING_DISTANCE_TOLERANCE_METERS = 0.06;
  private static final double APPROACH_DISTANCE_TOLERANCE_METERS = 0.12;
  private static final double NEAR_APPROACH_DISTANCE_TOLERANCE_METERS = 1.5;

  private static final double TARGET_ANGLE_TOLERANCE_DEG = 6.0;

  public static final double CORAL_MAX_VELOCITY_CONSTRAINTS = 1.6;
  public static final double CORAL_MAX_ACCELERATION_CONSTRAINTS = 0.4;

  public static final double NET_MAX_VELOCITY_CONSTRAINTS = 1.6;
  public static final double NET_MAX_ACCELERATION_CONSTRAINTS = 0.1;
  // elevator high consits of L4 and algae high
  public static final double ELV_HIGH_MAX_VELOCITY_CONSTRAINTS = 1.6;
  public static final double ELV_HIGH_MAX_ACCELERATION_CONSTRAINTS = 0.4;
  // elevator low consits of L1-3 and algae
  public static final double ELV_LOW_MAX_VELOCITY_CONSTRAINTS = 2.0;
  public static final double ELV_LOW_MAX_ACCELERATION_CONSTRAINTS = 1.0;

  public static TrapezoidProfile.Constraints m_coralXConstraint =
      new TrapezoidProfile.Constraints(
          CORAL_MAX_VELOCITY_CONSTRAINTS, CORAL_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_coralYConstraint =
      new TrapezoidProfile.Constraints(
          CORAL_MAX_VELOCITY_CONSTRAINTS, CORAL_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_coralThetaConstraint =
      new TrapezoidProfile.Constraints(
          CORAL_MAX_VELOCITY_CONSTRAINTS, CORAL_MAX_ACCELERATION_CONSTRAINTS);
  // net is the real net notnet something
  public static TrapezoidProfile.Constraints m_netXConstraint =
      new TrapezoidProfile.Constraints(
          NET_MAX_VELOCITY_CONSTRAINTS, NET_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_netYConstraint =
      new TrapezoidProfile.Constraints(
          NET_MAX_VELOCITY_CONSTRAINTS, NET_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_netThetaConstraint =
      new TrapezoidProfile.Constraints(
          NET_MAX_VELOCITY_CONSTRAINTS, NET_MAX_ACCELERATION_CONSTRAINTS);
  public static TrapezoidProfile.Constraints m_elvHighXConstraint =
      new TrapezoidProfile.Constraints(
          ELV_HIGH_MAX_VELOCITY_CONSTRAINTS, ELV_HIGH_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_elvHighYConstraint =
      new TrapezoidProfile.Constraints(
          ELV_HIGH_MAX_VELOCITY_CONSTRAINTS, ELV_HIGH_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_elvHighThetaConstraint =
      new TrapezoidProfile.Constraints(
          ELV_HIGH_MAX_VELOCITY_CONSTRAINTS, ELV_HIGH_MAX_ACCELERATION_CONSTRAINTS);
  public static TrapezoidProfile.Constraints m_elvLowXConstraint =
      new TrapezoidProfile.Constraints(
          ELV_LOW_MAX_VELOCITY_CONSTRAINTS, ELV_LOW_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_elvLowYConstraint =
      new TrapezoidProfile.Constraints(
          ELV_LOW_MAX_VELOCITY_CONSTRAINTS, ELV_LOW_MAX_ACCELERATION_CONSTRAINTS);

  public static TrapezoidProfile.Constraints m_elvLowThetaConstraint =
      new TrapezoidProfile.Constraints(
          ELV_LOW_MAX_VELOCITY_CONSTRAINTS, ELV_LOW_MAX_ACCELERATION_CONSTRAINTS);

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
  private ReefSide m_lastTargetReefSide = ReefSide.Left;

  private final AtomicBoolean m_readyToScore;
  private final AtomicBoolean m_readyToBackOff;

  private boolean m_finishedScoring = false;

  private TargetStage m_targetStage = TargetStage.MoveToApproach;

  private final RobotInfo.DriveInfo m_driveInfo;

  private final AtomicReference<Pose2d> m_providedPose;

  public enum ReefSide {
    Left,
    Right,
    Center,
    LevelOneLeft,
    LevelOneRight,
    LevelOneCenter
  }

  public enum ReefFace {
    A,
    B,
    C,
    D,
    E,
    F,
    Net,
    Processor
  }

  public enum TargetStage {
    MoveToApproach,
    Approach,
    MoveToScoring,
    Scoring,
    MoveToBackOff,
    BackOff
  }

  public static void getRobotInfo() {}

  public static class TargetPositions {
    private static final double REEF_WIDTH_METERS = 0.33;

    private static final Translation2d LEFT_REEF_APPROACH_TARGET =
        new Translation2d(-REEF_WIDTH_METERS / 2.0, 1.01);
    private static final Translation2d RIGHT_REEF_APPROACH_TARGET =
        new Translation2d(REEF_WIDTH_METERS / 2.0, 1.01);
    private static final Translation2d L_1_REEF_APPROACH_TARGET = new Translation2d(0.0, 1.01);

    private static final Translation2d ALGAE_APPROACH_TARGET = new Translation2d(0, 0.9);
    private static final double ALGAE_PICKUP_DIST = 0.55;

    private static final Translation2d HP_APPROACH_TARGET = new Translation2d(0.0, 0.5);
    private static final Translation2d PROCESSOR_APPROACH_TARGET = new Translation2d(0.0, 1);
    private static final Translation2d NET_APPROACH_TARGET = new Translation2d(0.5, 1.5);

    private static final double REEF_SCORING_DIST = 0.56;
    private static final double PROCESSOR_SCORING_DIST = 0.3175;
    private static final double NET_SCORING_DIST = 0.1;

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

    public static final TargetPositionRelativeToAprilTag PROCESSOR =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(3),
            PROCESSOR_APPROACH_TARGET,
            PROCESSOR_SCORING_DIST,
            new Rotation2d());

    public static final TargetPositionRelativeToAprilTag NET =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(5),
            NET_APPROACH_TARGET,
            NET_SCORING_DIST,
            Rotation2d.fromDegrees(180));

    public static final TargetPositionRelativeToAprilTag A_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag A_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());

    public static final double L1_OFFSET_ANGLE = 11.0;

    public static final TargetPositionRelativeToAprilTag A_L_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7),
            LEFT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag A_R_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7),
            RIGHT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(-L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag A_C_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), L_1_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag A_ALGAE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(7), ALGAE_APPROACH_TARGET, ALGAE_PICKUP_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag B_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag B_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag B_L_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8),
            LEFT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag B_R_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8),
            RIGHT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(-L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag B_C_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), L_1_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag B_ALGAE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(8), ALGAE_APPROACH_TARGET, ALGAE_PICKUP_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag C_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag C_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag C_L_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9),
            LEFT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag C_R_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9),
            RIGHT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(-L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag C_C_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), L_1_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag C_ALGAE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(9), ALGAE_APPROACH_TARGET, ALGAE_PICKUP_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag D_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag D_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag D_L_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10),
            LEFT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag D_R_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10),
            RIGHT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(-L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag D_C_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), L_1_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag D_ALGAE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(10), ALGAE_APPROACH_TARGET, ALGAE_PICKUP_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag E_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag E_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag E_L_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11),
            LEFT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag E_R_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11),
            RIGHT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(-L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag E_C_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), L_1_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag E_ALGAE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(11), ALGAE_APPROACH_TARGET, ALGAE_PICKUP_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag F_L =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), LEFT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag F_R =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), RIGHT_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag F_L_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6),
            LEFT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag F_R_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6),
            RIGHT_REEF_APPROACH_TARGET,
            REEF_SCORING_DIST,
            Rotation2d.fromDegrees(-L1_OFFSET_ANGLE));
    public static final TargetPositionRelativeToAprilTag F_C_L_1 =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), L_1_REEF_APPROACH_TARGET, REEF_SCORING_DIST, new Rotation2d());
    public static final TargetPositionRelativeToAprilTag F_ALGAE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(6), ALGAE_APPROACH_TARGET, ALGAE_PICKUP_DIST, new Rotation2d());
  }

  public DriveWithLimelight(
      Logger logger, AtomicBoolean readyToScore, AtomicBoolean readyToBackOff) {
    m_driveInfo = RobotConfig.get().DRIVE_INFO;
    double controlPeriodSeconds = 1.0 / m_driveInfo.STATUS_SIGNAL_FREQUENCY;
    m_xController = new ProfiledPIDController(4.0, 0, 0, m_coralXConstraint, controlPeriodSeconds);
    m_yController = new ProfiledPIDController(4.0, 0, 0, m_coralYConstraint, controlPeriodSeconds);
    m_thetaController =
        new ProfiledPIDController(8.0, 0, 0, m_coralThetaConstraint, controlPeriodSeconds);

    m_thetaController.enableContinuousInput(
        Units.degreesToRadians(0.0), Units.degreesToRadians(360.0));

    m_logger = logger;

    m_readyToScore = readyToScore;
    m_readyToBackOff = readyToBackOff;

    m_providedPose = new AtomicReference<>(new Pose2d());
  }

  public void setTargetSide(ReefSide side) {
    if (m_targetReefSide == ReefSide.Left || m_targetReefSide == ReefSide.Right) {
      m_lastTargetReefSide = m_targetReefSide;
    }

    m_targetReefSide = side;

    m_approachPoseLog = getTargetReefPosition().getApproachPose();
    m_scoringPoseLog = getTargetReefPosition().getScoringPose();
  }

  public void setTargetReefFace(ReefFace face) {
    m_targetReefFace = face;
    m_approachPoseLog = getTargetReefPosition().getApproachPose();
    m_scoringPoseLog = getTargetReefPosition().getScoringPose();
  }

  public ReefFace getTargetReefFace() {
    return m_targetReefFace;
  }

  public synchronized void setConstraints(ReefLevel level) {
    if (level == ReefLevel.Net) {
      m_xController.setConstraints(m_netXConstraint);
      m_yController.setConstraints(m_netYConstraint);
      m_thetaController.setConstraints(m_netThetaConstraint);
    } else if (level == ReefLevel.L_4 || level == ReefLevel.AlgaeHigh) {
      m_xController.setConstraints(m_elvHighXConstraint);
      m_yController.setConstraints(m_elvHighYConstraint);
      m_thetaController.setConstraints(m_netThetaConstraint);
    } else {
      m_xController.setConstraints(m_elvLowXConstraint);
      m_yController.setConstraints(m_elvLowYConstraint);
      m_thetaController.setConstraints(m_elvLowThetaConstraint);
    }
    m_logger.log("XcontrolerConstraintsV", m_xController.getConstraints().maxAcceleration);
    m_logger.log("XcontrolerConstraintsA", m_xController.getConstraints().maxVelocity);
  }

  private TargetPositionRelativeToAprilTag getPositionFromReefSide(
      ReefSide side,
      TargetPositionRelativeToAprilTag leftTarget,
      TargetPositionRelativeToAprilTag centerTarget,
      TargetPositionRelativeToAprilTag rightTarget,
      TargetPositionRelativeToAprilTag levelOneLeftTarget,
      TargetPositionRelativeToAprilTag levelOneRightTarget,
      TargetPositionRelativeToAprilTag levelOneCenterTarget) {
    switch (side) {
      case Left:
        return leftTarget;
      case Center:
        return centerTarget;
      case Right:
        return rightTarget;
      case LevelOneLeft:
        return levelOneLeftTarget;
      case LevelOneRight:
        return levelOneRightTarget;
      case LevelOneCenter:
        return levelOneCenterTarget;
      default:
        throw new IllegalArgumentException(side.toString());
    }
  }

  public TargetPositionRelativeToAprilTag getTargetReefPosition() {
    switch (m_targetReefFace) {
      case A:
        return getPositionFromReefSide(
            m_targetReefSide,
            TargetPositions.A_L,
            TargetPositions.A_ALGAE,
            TargetPositions.A_R,
            TargetPositions.A_L_L_1,
            TargetPositions.A_R_L_1,
            TargetPositions.A_C_L_1);
      case B:
        return getPositionFromReefSide(
            m_targetReefSide,
            TargetPositions.B_L,
            TargetPositions.B_ALGAE,
            TargetPositions.B_R,
            TargetPositions.B_L_L_1,
            TargetPositions.B_R_L_1,
            TargetPositions.B_C_L_1);
      case C:
        return getPositionFromReefSide(
            m_targetReefSide,
            TargetPositions.C_L,
            TargetPositions.C_ALGAE,
            TargetPositions.C_R,
            TargetPositions.C_L_L_1,
            TargetPositions.C_R_L_1,
            TargetPositions.C_C_L_1);
      case D:
        return getPositionFromReefSide(
            m_targetReefSide,
            TargetPositions.D_L,
            TargetPositions.D_ALGAE,
            TargetPositions.D_R,
            TargetPositions.D_L_L_1,
            TargetPositions.D_R_L_1,
            TargetPositions.D_C_L_1);
      case E:
        return getPositionFromReefSide(
            m_targetReefSide,
            TargetPositions.E_L,
            TargetPositions.E_ALGAE,
            TargetPositions.E_R,
            TargetPositions.E_L_L_1,
            TargetPositions.E_R_L_1,
            TargetPositions.E_C_L_1);
      case F:
        return getPositionFromReefSide(
            m_targetReefSide,
            TargetPositions.F_L,
            TargetPositions.F_ALGAE,
            TargetPositions.F_R,
            TargetPositions.F_L_L_1,
            TargetPositions.F_R_L_1,
            TargetPositions.F_C_L_1);
      case Processor:
        return TargetPositions.PROCESSOR;
      case Net:
        return TargetPositions.NET;
      default:
        throw new IllegalArgumentException("Invalid reef face: " + m_targetReefFace);
    }
  }

  public synchronized TargetStage getTargetStage() {
    return m_targetStage;
  }

  public void targetReefPosition() {
    if (getTargetReefPosition() != m_target) {
      m_approachPose = getTargetReefPosition().getApproachPose();
      m_scoringPose = getTargetReefPosition().getScoringPose();

      m_approachPoseLog = getTargetReefPosition().getApproachPose();
      m_scoringPoseLog = getTargetReefPosition().getScoringPose();

      m_target = getTargetReefPosition();

      m_targetStage = TargetStage.MoveToApproach;
    }
  }

  public void log() {
    SmartDashboard.putString("DB/String 6", "Reef Face: " + m_targetReefFace);
    SmartDashboard.putString("DB/String 7", "Reef Side: " + m_targetReefSide);

    m_logger.log("Target Side", m_targetReefSide.toString());
    m_logger.log("Target Stage", m_targetStage.toString());

    m_logger.log("Is At Approach", isAtApproach(m_providedPose.get()));
    m_logger.log("Is At Scoring", isAtScoring(m_providedPose.get()));

    m_logger.log("Approach Error/X", m_providedPose.get().getX() - m_approachPose.getX());
    m_logger.log("Approach Error/Y", m_providedPose.get().getY() - m_approachPose.getY());
    m_logger.log(
        "Approach Error/Deg",
        m_providedPose.get().getRotation().getDegrees()
            - m_approachPose.getRotation().getDegrees());

    m_logger.log("Scoring Error/X", m_providedPose.get().getX() - m_scoringPose.getX());
    m_logger.log("Scoring Error/Y", m_providedPose.get().getY() - m_scoringPose.getY());
    m_logger.log(
        "Scoring Error/Deg",
        m_providedPose.get().getRotation().getDegrees() - m_scoringPose.getRotation().getDegrees());

    m_logger.log("Target Approach Pose", m_approachPose);

    m_logger.log("Target Approach Pose Log", m_approachPoseLog);

    m_logger.log("Target Scoring Pose", m_scoringPose);

    m_logger.log(
        "Controller Target Position",
        new Pose2d(
            m_xController.getSetpoint().position,
            m_yController.getSetpoint().position,
            Rotation2d.fromDegrees(m_thetaController.getSetpoint().position)));
    m_logger.log("x-state/velocity", m_xController.getSetpoint().velocity);
    m_logger.log("x-state/position", m_xController.getSetpoint().position);
    m_logger.log("y-state/velocity", m_yController.getSetpoint().velocity);
    m_logger.log("y-state/position", m_yController.getSetpoint().position);

    m_logger.log("Target Scoring Pose Log", m_scoringPoseLog);
    Pose2d currentTargetPose2d = getCurrentTargetPose2d();
    m_logger.log(
        "dist to target",
        m_providedPose.get().getTranslation().getDistance(currentTargetPose2d.getTranslation()));
  }

  public boolean isNearApproach(Pose2d currentPose) {
    return Drive.comparePoses(
        currentPose,
        m_approachPose,
        NEAR_APPROACH_DISTANCE_TOLERANCE_METERS,
        TARGET_ANGLE_TOLERANCE_DEG);
  }

  private boolean isAtApproach(Pose2d currentPose) {
    return Drive.comparePoses(
        currentPose,
        m_approachPose,
        APPROACH_DISTANCE_TOLERANCE_METERS,
        TARGET_ANGLE_TOLERANCE_DEG);
  }

  private boolean isAtScoring(Pose2d currentPose) {
    return Drive.comparePoses(
        currentPose, m_scoringPose, SCORING_DISTANCE_TOLERANCE_METERS, TARGET_ANGLE_TOLERANCE_DEG);
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

  public void toggleReefSide() {
    if (m_targetReefSide == ReefSide.Left) {
      setTargetSide(ReefSide.Right);
    } else if (m_targetReefSide == ReefSide.Right) {
      setTargetSide(ReefSide.Left);
    }
  }

  public ReefSide getLastTargetReefSide() {
    return m_lastTargetReefSide;
  }

  public synchronized void init(
      ChassisSpeeds previousChassisSpeeds, boolean robotIsAutonomous, Pose2d currentPose) {
    if (robotIsAutonomous) {
      m_xController.reset(currentPose.getX(), previousChassisSpeeds.vxMetersPerSecond);
      m_yController.reset(currentPose.getY(), previousChassisSpeeds.vyMetersPerSecond);
      m_thetaController.reset(
          currentPose.getRotation().getRadians(), previousChassisSpeeds.omegaRadiansPerSecond);
    } else {
      m_xController.reset(currentPose.getX());
      m_yController.reset(currentPose.getY());
      m_thetaController.reset(currentPose.getRotation().getRadians());
    }

    if (m_finishedScoring) {
      m_targetStage = TargetStage.MoveToApproach;
      m_finishedScoring = false;
    }
  }

  public void exit() {
    if (m_finishedScoring) {
      toggleReefSide();
    }
  }

  public synchronized ChassisSpeeds getOutput(Pose2d currentPose, Rotation2d angularVelocity) {
    if (m_target == null) {
      return new ChassisSpeeds(0, 0, 0);
    }

    switch (m_targetStage) {
      case MoveToApproach:
        if (isAtApproach(currentPose)) {
          m_targetStage = TargetStage.Approach;
        }
        break;
      case Approach:
        if (m_readyToScore.get()) {
          m_targetStage = TargetStage.MoveToScoring;
        }
        break;
      case MoveToScoring:
        if (isAtScoring(currentPose)) {
          m_targetStage = TargetStage.Scoring;
        }
        break;
      case Scoring:
        if (m_readyToBackOff.get()) {
          m_targetStage = TargetStage.MoveToBackOff;
          m_finishedScoring = true;
        }
        break;
      case MoveToBackOff:
        if (isAtApproach(currentPose)) {
          m_targetStage = TargetStage.BackOff;
        }
        break;
      case BackOff:
        break;
    }

    m_providedPose.set(currentPose);

    return new ChassisSpeeds(
        m_xController.calculate(currentPose.getX(), getCurrentTargetPose2d().getX())
            + m_xController.getSetpoint().velocity,
        m_yController.calculate(currentPose.getY(), getCurrentTargetPose2d().getY())
            + m_yController.getSetpoint().velocity,
        m_thetaController.calculate(
                currentPose.getRotation().getRadians(),
                getCurrentTargetPose2d().getRotation().getRadians())
            + m_thetaController.getSetpoint().velocity);
  }
}
