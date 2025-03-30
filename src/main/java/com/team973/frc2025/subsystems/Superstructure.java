package com.team973.frc2025.subsystems;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.ArmInfo;
import com.team973.frc2025.shared.RobotInfo.ElevatorInfo;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure implements Subsystem {
  private final Claw m_claw;
  private final Climb m_climb;
  private final Elevator m_elevator;
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final DriveController m_driveController;

  private final Logger m_logger;

  private static final ArmivatorPose.Config ARMIVATOR_CONFIG =
      new ArmivatorPose.Config(
          ArmInfo.ARM_LENGTH_METERS,
          ArmInfo.ARM_MAX_ANGLE_DEG,
          ArmInfo.ARM_MIN_ANGLE_DEG,
          ElevatorInfo.MAX_HEIGHT_METERS);

  private State m_state = State.Manual;
  private State m_lastState = State.Manual;

  private GamePiece m_gamePieceMode = GamePiece.Coral;

  private ReefLevel m_targetReefLevel = ReefLevel.L_1;

  private boolean m_manualScore = false;
  private boolean m_manualIntake = true;
  private boolean m_manualArmivator = false;

  private final SolidSignaler m_armTargetOutOfBoundsSignaler =
      new SolidSignaler(
          RobotInfo.Colors.PINK, 250, RobotInfo.SignalerInfo.ARM_TARGET_OUT_OF_BOUNDS_PRIORITY);

  public enum State {
    Manual,
    Score,
    Zero,
    Climb,
    Off
  }

  public enum ReefLevel {
    L_1(0.514),
    L_2(0.81),
    L_3(1.21),
    L_4(1.83),
    AlgaeHigh(0),
    AlgaeLow(0),
    Horizontal(0);

    private final double height;

    private ReefLevel(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }

  public enum GamePiece {
    Coral,
    Algae
  }

  public static class ArmivatorPose {
    private final double m_elevatorHeightMeters;
    private final double m_armAngleDeg;
    private final boolean m_targetIsOutOfBounds;

    public static class Config {
      public final double armLengthMeters;
      public final double maxArmAngleDeg;
      public final double minArmAngleDeg;
      public final double maxElevatorHeightMeters;

      public Config(
          double armLengthMeters,
          double maxArmAngleDeg,
          double minArmAngleDeg,
          double maxElevatorHeightMeters) {
        this.armLengthMeters = armLengthMeters;
        this.maxArmAngleDeg = maxArmAngleDeg;
        this.minArmAngleDeg = minArmAngleDeg;
        this.maxElevatorHeightMeters = maxElevatorHeightMeters;
      }
    }

    public ArmivatorPose(
        double elevatorHeightMeters, double armAngleDeg, boolean targetIsOutOfBounds) {
      m_elevatorHeightMeters = elevatorHeightMeters;
      m_armAngleDeg = armAngleDeg;
      m_targetIsOutOfBounds = targetIsOutOfBounds;
    }

    public double getElevatorHeightMeters() {
      return m_elevatorHeightMeters;
    }

    public double getElevatorHeightInches() {
      return getElevatorHeightMeters() * Conversions.Distance.INCHES_PER_METER;
    }

    public double getArmAngleDeg() {
      return m_armAngleDeg;
    }

    public boolean getTargetIsOutOfBounds() {
      return m_targetIsOutOfBounds;
    }

    public static ArmivatorPose fromCoordinate(double x, double y, Config config) {
      if (x > config.armLengthMeters) {
        x = config.armLengthMeters;
      } else if (x < 0) {
        x = 0;
      }

      double maxTargetHeight =
          config.maxElevatorHeightMeters
              + Math.sin(Math.toRadians(config.maxArmAngleDeg)) * config.armLengthMeters;
      double minTargetHeight =
          Math.sin(Math.toRadians(config.minArmAngleDeg)) * config.armLengthMeters;

      if (y > maxTargetHeight) {
        y = maxTargetHeight;
      } else if (y < minTargetHeight) {
        y = minTargetHeight;
      }

      double armAngleDeg = Math.toDegrees(Math.acos(x / config.armLengthMeters));
      double elevatorHeight;

      double upperElevator = y + (Math.sin(Math.toRadians(armAngleDeg)) * config.armLengthMeters);
      double lowerElevator = y - (Math.sin(Math.toRadians(armAngleDeg)) * config.armLengthMeters);

      if (lowerElevator < 0) {
        elevatorHeight = upperElevator;
        armAngleDeg *= -1;
      } else {
        elevatorHeight = lowerElevator;
      }

      boolean targetIsOutOfBounds = true;

      if (armAngleDeg > config.maxArmAngleDeg) {
        armAngleDeg = config.maxArmAngleDeg;
      } else if (armAngleDeg < config.minArmAngleDeg) {
        armAngleDeg = config.minArmAngleDeg;
      } else {
        targetIsOutOfBounds = false;
      }

      if (elevatorHeight > config.maxElevatorHeightMeters) {
        elevatorHeight = config.maxElevatorHeightMeters;
      } else if (elevatorHeight < 0) {
        elevatorHeight = 0;
      }

      return new ArmivatorPose(elevatorHeight, armAngleDeg, targetIsOutOfBounds);
    }

    public String toString() {
      return String.format(
          "ArmivatorPose(Arm=%fdeg,Elevator=%fm)", getArmAngleDeg(), getElevatorHeightInches());
    }
  }

  public static double getWristAngleRelativeToFloorDeg(double wristAngle, double armAngleDeg) {
    return wristAngle + armAngleDeg + 90.0;
  }

  public Superstructure(
      Claw claw,
      Climb climb,
      Elevator elevator,
      Arm arm,
      Wrist wrist,
      DriveController driveController,
      Logger logger) {
    m_claw = claw;
    m_climb = climb;
    m_elevator = elevator;
    m_arm = arm;
    m_wrist = wrist;
    m_driveController = driveController;
    m_logger = logger;
  }

  public void setState(State state) {
    m_state = state;
  }

  // public double setStowMode() {
  //   if (m_claw.getSeesCoral()){
  //     return
  //   }
  // }

  public void setManualScore(boolean score) {
    m_manualScore = score;
  }

  public void setManualIntake(boolean intake) {
    m_manualIntake = intake;
  }

  public void setTargetReefLevel(ReefLevel coralLevel, ReefLevel algaeLevel) {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_targetReefLevel = coralLevel;
    } else {
      m_targetReefLevel = algaeLevel;
    }
  }

  public void setTargetReefLevel(ReefLevel level) {
    m_targetReefLevel = level;
  }

  public boolean readyToScore() {
    if (m_arm.getTargetPosition() == Arm.CORAL_STOW_POSITION_DEG) {
      return false;
    }

    if (m_arm.getTargetPosition() == Arm.ALGAE_STOW_POSITION_DEG) {
      return false;
    }

    if (m_elevator.getTargetPosition() == Elevator.Presets.CORAL_STOW) {
      return false;
    }

    if (m_elevator.getTargetPosition() == Elevator.Presets.ALGAE_STOW) {
      return false;
    }

    if (m_wrist.getTargetPosition() == Wrist.WITH_CORAL_STOW_POSTION_DEG
        || m_wrist.getTargetPosition() == Wrist.WITHOUT_CORAL_STOW_POSITION_DEG) {
      return false;
    }

    if (m_wrist.getTargetPosition() == Wrist.ALGAE_STOW_POSITION_DEG) {
      return false;
    }

    if (!m_arm.motorAtTargetRotation()) {
      return false;
    }

    if (!m_elevator.motorAtTarget()) {
      return false;
    }

    if (!m_wrist.motorAtTargetRotation()) {
      return false;
    }

    return true;
  }

  public boolean getSeesCoral() {
    return m_claw.getSeesCoral();
  }

  public boolean readyToBackOff() {
    if (m_gamePieceMode == GamePiece.Coral) {
      return !getSeesCoral();
    }

    return m_claw.getHasAlgae();
  }

  public void log() {
    SmartDashboard.putString("DB/String 0", "Reef Level: " + m_targetReefLevel);
    SmartDashboard.putString(
        "DB/String 1",
        "E: " + String.valueOf(m_elevator.getTargetPositionFromLevel(m_targetReefLevel)));
    SmartDashboard.putString(
        "DB/String 2", "A: " + String.valueOf(m_arm.getTargetDegFromLevel(m_targetReefLevel)));
    SmartDashboard.putString("DB/String 8", m_gamePieceMode.toString());

    m_logger.log("State", m_state.toString());
    m_logger.log(
        "Wrist Angle Relative To Floor",
        getWristAngleRelativeToFloorDeg(m_wrist.getWristPostionDeg(), m_arm.getArmPostionDeg()));
    m_logger.log("Target Arm Angle Deg", getTargetArmivatorPose().getArmAngleDeg());
    m_logger.log(
        "Target Elevator Height Inches", getTargetArmivatorPose().getElevatorHeightInches());
    m_logger.log(
        "Armivator X Target", m_driveController.getDriveWithLimelight().getDistFromAprilTag());
    m_logger.log(
        "Armivator Y Target",
        m_targetReefLevel.getHeight() - ElevatorInfo.FLOOR_TO_ELEVATOR_ZERO_METERS);
    m_logger.log(
        "Target Wrist Angle Relative To Floor",
        getWristAngleRelativeToFloorDeg(m_wrist.getTargetPosition(), m_arm.getArmPostionDeg()));

    m_claw.log();
    m_climb.log();
    m_elevator.log();
    m_arm.log();
    m_wrist.log();
  }

  public void syncSensors() {
    m_claw.syncSensors();
    m_climb.syncSensors();
    m_elevator.syncSensors();
    m_arm.syncSensors();
    m_wrist.syncSensors();
  }

  private void armTargetReefLevel() {
    m_arm.setTargetDeg(m_arm.getTargetDegFromLevel(m_targetReefLevel));
    m_arm.setControlStatus(Arm.ControlStatus.TargetPostion);
  }

  private void armStow() {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_arm.setTargetDeg(Arm.CORAL_STOW_POSITION_DEG);
    } else {
      m_arm.setTargetDeg(Arm.ALGAE_STOW_POSITION_DEG);
    }
    m_arm.setControlStatus(Arm.ControlStatus.TargetPostion);
  }

  private void elevatorTargetReefLevel() {
    m_elevator.setTargetPostion(m_elevator.getTargetPositionFromLevel(m_targetReefLevel));
    m_elevator.setControlStatus(Elevator.ControlStatus.TargetPostion);
  }

  private void elevatorStow() {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_elevator.setTargetPostion(Elevator.Presets.CORAL_STOW);
    } else {
      m_elevator.setTargetPostion(Elevator.Presets.ALGAE_STOW);
    }
    m_elevator.setControlStatus(Elevator.ControlStatus.TargetPostion);
  }

  private void wristStow() {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_wrist.setTargetDeg(Wrist.WITHOUT_CORAL_STOW_POSITION_DEG);
    } else {
      m_wrist.setTargetDeg(Wrist.ALGAE_STOW_POSITION_DEG);
    }
    m_wrist.setControlStatus(Wrist.ControlStatus.TargetPostion);
  }

  private void wristTargetReefLevel(boolean relativeToArm) {
    if (relativeToArm) {
      m_wrist.setTargetDeg(m_wrist.getTargetDegFromLevel(m_targetReefLevel));
    } else {
      m_wrist.setTargetDeg(
          getWristAngleRelativeToFloorDeg(
              m_wrist.getTargetDegFromLevel(m_targetReefLevel), m_arm.getArmPostionDeg()));
    }

    m_wrist.setControlStatus(Wrist.ControlStatus.TargetPostion);
  }

  public void incrementWristOffset(double increment) {
    m_wrist.incrementOffset(increment, m_targetReefLevel);
  }

  public void incrementArmOffset(double increment) {
    m_arm.incrementOffset(increment, m_targetReefLevel);
  }

  public void setManualArmivator(boolean manual) {
    m_manualArmivator = manual;
  }

  public void incrementElevatorOffset(double increment) {
    m_elevator.incrementOffset(increment, m_targetReefLevel);
  }

  public void setGamePieceMode(GamePiece gamePiece) {
    m_gamePieceMode = gamePiece;
  }

  public void toggleGamePieceMode() {
    if (m_gamePieceMode == GamePiece.Coral) {
      setGamePieceMode(GamePiece.Algae);
    } else {
      setGamePieceMode(GamePiece.Coral);
    }
  }

  public void incrementCoralBackup(double increment) {
    m_claw.incrementBackup(increment);
  }

  public void incrementClimbTarget(double increment) {
    m_climb.incrementTarget(increment);
  }

  public void setClimbTarget(double target) {
    m_climb.setTarget(target);
  }

  public void clawIntake() {
    if (!m_manualIntake) {
      m_claw.setControl(Claw.ControlStatus.Off);
    } else if (m_gamePieceMode == GamePiece.Coral) {
      m_claw.setControl(Claw.ControlStatus.IntakeCoral);
    } else {
      m_claw.setControl(Claw.ControlStatus.IntakeAlgae);
    }
  }

  public void clawScore() {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_claw.setControl(Claw.ControlStatus.ScoreCoral);
    } else {
      m_claw.setControl(Claw.ControlStatus.ScoreAlgae);
    }
  }

  public ArmivatorPose getTargetArmivatorPose() {
    double x =
        m_driveController.getDriveWithLimelight().getDistFromAprilTag()
            - ArmInfo.ROBOT_CENTER_TO_ARM_PIVOT_METERS;
    double y = m_targetReefLevel.getHeight() - ElevatorInfo.FLOOR_TO_ELEVATOR_ZERO_METERS;

    return ArmivatorPose.fromCoordinate(x, y, ARMIVATOR_CONFIG);
  }

  public void armivatorTargetReef() {
    m_arm.setTargetDeg(getTargetArmivatorPose().getArmAngleDeg());
    m_elevator.setTargetPostion(getTargetArmivatorPose().getElevatorHeightInches());

    if (getTargetArmivatorPose().getTargetIsOutOfBounds()) {
      m_armTargetOutOfBoundsSignaler.enable();
    } else {
      m_armTargetOutOfBoundsSignaler.disable();
    }
  }

  public void update() {
    switch (m_state) {
      case Manual:
        if (m_lastState != m_state) {
          m_manualScore = false;
          m_manualIntake = true;
        }

        if (m_manualScore) {
          clawScore();
        } else if (m_manualIntake) {
          clawIntake();
        } else {
          m_claw.setControl(Claw.ControlStatus.Off);
        }

        if (m_manualArmivator) {
          armTargetReefLevel();
          elevatorTargetReefLevel();
          wristTargetReefLevel(true);
        } else {
          armStow();
          elevatorStow();
          wristStow();
        }

        break;
      case Score:
        clawIntake();

        if (m_lastState != m_state) {
          m_manualScore = false;
        }

        switch (m_driveController.getDriveWithLimelight().getTargetStage()) {
          case MoveToApproach:
            if (m_driveController.getDriveWithLimelight().isNearApproach()) {
              armivatorTargetReef();
              wristTargetReefLevel(false);

              m_manualArmivator = true;
            } else {
              armStow();
              elevatorStow();
              wristStow();

              m_manualArmivator = false;
            }
            break;
          case Approach:
            armivatorTargetReef();
            wristTargetReefLevel(false);

            m_manualArmivator = true;
            break;
          case MoveToScoring:
            armivatorTargetReef();
            wristTargetReefLevel(false);
            break;
          case Scoring:
            if (m_manualScore) {
              clawScore();
            }

            armivatorTargetReef();
            wristTargetReefLevel(false);
            break;
          case MoveToBackOff:
            m_claw.setControl(Claw.ControlStatus.Off);

            armivatorTargetReef();
            wristTargetReefLevel(false);
            break;
          case BackOff:
            armStow();
            elevatorStow();
            wristStow();

            m_manualArmivator = false;
            break;
        }
        break;
      case Zero:
        m_arm.setControlStatus(Arm.ControlStatus.Zero);
        m_elevator.setControlStatus(Elevator.ControlStatus.Zero);
        m_wrist.setControlStatus(Wrist.ControlStatus.Zero);

        clawIntake();
        break;
      case Climb:
        m_claw.setControl(Claw.ControlStatus.Off);

        setTargetReefLevel(ReefLevel.Horizontal);

        armTargetReefLevel();
        elevatorTargetReefLevel();
        wristTargetReefLevel(true);

        m_manualArmivator = true;
        break;
      case Off:
        m_claw.setControl(Claw.ControlStatus.Off);
        m_arm.setControlStatus(Arm.ControlStatus.Off);
        m_elevator.setControlStatus(Elevator.ControlStatus.Off);
        m_wrist.setControlStatus(Wrist.ControlStatus.Off);
        break;
    }

    m_lastState = m_state;

    m_claw.update();
    m_climb.update();
    m_elevator.update();
    m_arm.update();
    m_wrist.update();
  }

  public void reset() {
    m_claw.reset();
    m_climb.reset();
    m_elevator.reset();
    m_arm.reset();
    m_wrist.reset();
  }
}
