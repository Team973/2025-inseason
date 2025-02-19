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
  private static final double TARGET_DISTANCE_TOLERANCE_METERS = 0.03;
  private static final double TARGET_ANGLE_TOLERANCE_DEG = 5.0;

  private final GreyPoseEstimator m_poseEstimator;

  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private final Logger m_logger;

  private TargetPositionRelativeToAprilTag m_target = null;

  private Pose2d m_targetInitialPose = new Pose2d();
  private Pose2d m_targetFinalPose = new Pose2d();

  private Pose2d m_targetInitialPoseLog = new Pose2d();
  private Pose2d m_targetFinalPoseLog = new Pose2d();

  private int m_targetReefFace = 1;
  private TargetReefSide m_targetReefSide = TargetReefSide.Left;

  private BooleanSupplier m_targetFinalPoseGate = () -> true;
  private BooleanSupplier m_reTargetInitialPoseGate = () -> false;

  private TargetMode m_targetMode = TargetMode.Initial;

  private boolean m_targetingComplete = false;

  public enum TargetReefSide {
    Left,
    Right
  }

  private enum TargetMode {
    Initial,
    Final,
    ReInitial
  }

  public static class TargetPositions {
    private static final double REEF_WIDTH_METERS = 0.33;
    private static final Translation2d LEFT_REEF_INITIAL_TARGET =
        new Translation2d(REEF_WIDTH_METERS / 2.0, 0.8);
    private static final Translation2d RIGHT_REEF_INITIAL_TARGET =
        new Translation2d(-REEF_WIDTH_METERS / 2.0, 0.8);
    private static final double REEF_FINAL_DIST = 0.1;

    private static final Translation2d HP_INITIAL_TARGET = new Translation2d(0.0, 0.5);

    public static final TargetPositionRelativeToAprilTag TEST_ONE =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(0), new Translation2d(0.1, 0.2), 0, Rotation2d.fromDegrees(180));
    public static final TargetPositionRelativeToAprilTag TEST_TWO =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(-1), new Translation2d(0.1, 0.3), 0.1, new Rotation2d());

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

    double controlPeriodSeconds = 1.0 / RobotInfo.DriveInfo.STATUS_SIGNAL_FREQUENCY;
    m_xController =
        new ProfiledPIDController(
            5.0, 0, 0, new TrapezoidProfile.Constraints(1.0, 0.25), controlPeriodSeconds);
    m_yController =
        new ProfiledPIDController(
            5.0, 0, 0, new TrapezoidProfile.Constraints(1.0, 0.25), controlPeriodSeconds);
    m_thetaController =
        new ProfiledPIDController(
            8.0, 0, 0, new TrapezoidProfile.Constraints(1.0, 0.6), controlPeriodSeconds);

    m_thetaController.enableContinuousInput(
        Units.degreesToRadians(0.0), Units.degreesToRadians(360.0));

    m_logger = logger;
  }

  public void setTargetSide(TargetReefSide side) {
    m_targetReefSide = side;

    m_targetInitialPoseLog = getTargetReefPosition().getInitialTargetPose();
    m_targetFinalPoseLog = getTargetReefPosition().getFinalTargetPose();
  }

  public void setTargetReefFace(int face) {
    m_targetReefFace = face;

    if (m_targetReefFace > 6) {
      m_targetReefFace = 6;
    } else if (m_targetReefFace < 1) {
      m_targetReefFace = 1;
    }

    m_targetInitialPoseLog = getTargetReefPosition().getInitialTargetPose();
    m_targetFinalPoseLog = getTargetReefPosition().getFinalTargetPose();
  }

  public TargetPositionRelativeToAprilTag getTargetReefPosition() {
    switch (m_targetReefFace) {
      case 1:
        return m_targetReefSide == TargetReefSide.Left
            ? TargetPositions.ONE_L
            : TargetPositions.ONE_R;
      case 2:
        return m_targetReefSide == TargetReefSide.Left
            ? TargetPositions.TWO_L
            : TargetPositions.TWO_R;
      case 3:
        return m_targetReefSide == TargetReefSide.Left
            ? TargetPositions.THREE_L
            : TargetPositions.THREE_R;
      case 4:
        return m_targetReefSide == TargetReefSide.Left
            ? TargetPositions.FOUR_L
            : TargetPositions.FOUR_R;
      case 5:
        return m_targetReefSide == TargetReefSide.Left
            ? TargetPositions.FIVE_L
            : TargetPositions.FIVE_R;
      case 6:
        return m_targetReefSide == TargetReefSide.Left
            ? TargetPositions.SIX_L
            : TargetPositions.SIX_R;
      default:
        throw new IllegalArgumentException("Invalid reef face: " + m_targetReefFace);
    }
  }

  private void setTargetMode(TargetMode targetMode) {
    if (targetMode == TargetMode.Final && m_targetFinalPoseGate.getAsBoolean()) {
      m_targetMode = TargetMode.Final;
    } else if (targetMode == TargetMode.ReInitial && m_reTargetInitialPoseGate.getAsBoolean()) {
      m_targetMode = TargetMode.ReInitial;
    } else if (targetMode == TargetMode.Initial) {
      m_targetMode = TargetMode.Initial;
    }
  }

  public void targetReefPosition(
      BooleanSupplier targetFinalPoseGate, BooleanSupplier reTargetInitialPoseGate) {
    if (getTargetReefPosition() != m_target) {
      m_targetInitialPose = getTargetReefPosition().getInitialTargetPose();
      m_targetFinalPose = getTargetReefPosition().getFinalTargetPose();

      m_targetInitialPoseLog = getTargetReefPosition().getInitialTargetPose();
      m_targetFinalPoseLog = getTargetReefPosition().getFinalTargetPose();

      setTargetMode(TargetMode.Initial);
      m_target = getTargetReefPosition();
      m_targetingComplete = false;
    }

    m_targetFinalPoseGate = targetFinalPoseGate;
    m_reTargetInitialPoseGate = reTargetInitialPoseGate;
  }

  public void log() {
    SmartDashboard.putString("DB/String 6", "Reef Face: " + m_targetReefFace);
    SmartDashboard.putString("DB/String 7", "Reef Side: " + m_targetReefSide);

    m_logger.log("Target Mode", m_targetMode.toString());
    m_logger.log("Target Side", m_targetReefSide.toString());

    m_logger.log(
        "Target Initial Pose",
        new double[] {
          m_targetInitialPose.getX(),
          m_targetInitialPose.getY(),
          m_targetInitialPose.getRotation().getRadians()
        });

    m_logger.log(
        "Target Initial Pose Log",
        new double[] {
          m_targetInitialPoseLog.getX(),
          m_targetInitialPoseLog.getY(),
          m_targetInitialPoseLog.getRotation().getRadians()
        });

    m_logger.log(
        "Target Final Pose",
        new double[] {
          m_targetFinalPose.getX(),
          m_targetFinalPose.getY(),
          m_targetFinalPose.getRotation().getRadians()
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
          m_targetFinalPoseLog.getX(),
          m_targetFinalPoseLog.getY(),
          m_targetFinalPoseLog.getRotation().getRadians()
        });
  }

  public boolean reachedTargetInitialPose() {
    return Drive.comparePoses(
        m_poseEstimator.getPoseMeters(),
        m_targetInitialPose,
        TARGET_DISTANCE_TOLERANCE_METERS,
        TARGET_ANGLE_TOLERANCE_DEG);
  }

  public boolean reachedTargetFinalPose() {
    return Drive.comparePoses(
        m_poseEstimator.getPoseMeters(),
        m_targetFinalPose,
        TARGET_DISTANCE_TOLERANCE_METERS,
        TARGET_ANGLE_TOLERANCE_DEG);
  }

  public boolean getTargetingComplete() {
    return m_targetingComplete;
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

    if (reachedTargetInitialPose() && m_targetMode == TargetMode.ReInitial) {
      m_targetingComplete = true;
    } else if (reachedTargetInitialPose()) {
      setTargetMode(TargetMode.Final);
    } else if (reachedTargetFinalPose()) {
      setTargetMode(TargetMode.ReInitial);
    }

    switch (m_targetMode) {
      case ReInitial:
      case Initial:
        return new ChassisSpeeds(
            m_xController.calculate(
                    m_poseEstimator.getPoseMeters().getX(), m_targetInitialPose.getX())
                + m_xController.getSetpoint().velocity,
            m_yController.calculate(
                    m_poseEstimator.getPoseMeters().getY(), m_targetInitialPose.getY())
                + m_yController.getSetpoint().velocity,
            m_thetaController.calculate(
                    m_poseEstimator.getPoseMeters().getRotation().getRadians(),
                    m_targetInitialPose.getRotation().getRadians())
                + m_thetaController.getSetpoint().velocity);
      case Final:
        return new ChassisSpeeds(
            m_xController.calculate(
                    m_poseEstimator.getPoseMeters().getX(), m_targetFinalPose.getX())
                + m_xController.getSetpoint().velocity,
            m_yController.calculate(
                    m_poseEstimator.getPoseMeters().getY(), m_targetFinalPose.getY())
                + m_yController.getSetpoint().velocity,
            m_thetaController.calculate(
                    m_poseEstimator.getPoseMeters().getRotation().getRadians(),
                    m_targetFinalPose.getRotation().getRadians())
                + m_thetaController.getSetpoint().velocity);
      default:
        throw new IllegalArgumentException(m_targetMode.toString());
    }
  }
}
