package com.team973.frc2025.subsystems;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.ArmInfo;
import com.team973.frc2025.shared.RobotInfo.ElevatorInfo;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure implements Subsystem {
  private final Claw m_claw;
  private final Climb m_climb;
  private final Elevator m_elevator;
  private final Arm m_arm;
  private final DriveController m_driveController;

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
    private final double m_elevatorHeight;
    private final double m_armAngle;
    private final boolean m_targetIsOutOfBounds;

    public static class Config {
      public final double armLength;
      public final double maxArmAngle;
      public final double minArmAngle;
      public final double maxElevatorHeight;

      public Config(
          double armLength, double maxArmAngle, double minArmAngle, double maxElevatorHeight) {
        this.armLength = armLength;
        this.maxArmAngle = maxArmAngle;
        this.minArmAngle = minArmAngle;
        this.maxElevatorHeight = maxElevatorHeight;
      }
    }

    public ArmivatorPose(double elevatorHeight, double armAngle, boolean targetIsOutOfBounds) {
      m_elevatorHeight = elevatorHeight;
      m_armAngle = armAngle;
      m_targetIsOutOfBounds = targetIsOutOfBounds;
    }

    public double getElevatorHeight() {
      return m_elevatorHeight;
    }

    public double getArmAngle() {
      return m_armAngle;
    }

    public boolean getTargetIsOutOfBounds() {
      return m_targetIsOutOfBounds;
    }

    public static ArmivatorPose fromCoordinate(double x, double y, Config config) {
      if (x > config.armLength) {
        x = config.armLength;
      } else if (x < 0) {
        x = 0;
      }

      double maxTargetHeight =
          config.maxElevatorHeight
              + Math.sin(Math.toRadians(config.maxArmAngle)) * config.armLength;
      double minTargetHeight = Math.sin(Math.toRadians(config.minArmAngle)) * config.armLength;

      if (y > maxTargetHeight) {
        y = maxTargetHeight;
      } else if (y < minTargetHeight) {
        y = minTargetHeight;
      }

      double armAngleDeg = Math.toDegrees(Math.acos(x / config.armLength));
      double elevatorHeight;

      double upperElevator = y + (Math.sin(Math.toRadians(armAngleDeg)) * config.armLength);
      double lowerElevator = y - (Math.sin(Math.toRadians(armAngleDeg)) * config.armLength);

      if (lowerElevator < 0) {
        elevatorHeight = upperElevator;
        armAngleDeg *= -1;
      } else {
        elevatorHeight = lowerElevator;
      }

      boolean targetIsOutOfBounds = true;

      if (armAngleDeg > config.maxArmAngle) {
        armAngleDeg = config.maxArmAngle;
      } else if (armAngleDeg < config.minArmAngle) {
        armAngleDeg = config.minArmAngle;
      } else {
        targetIsOutOfBounds = false;
      }

      return new ArmivatorPose(elevatorHeight, armAngleDeg, targetIsOutOfBounds);
    }
  }

  public Superstructure(
      Claw claw, Climb climb, Elevator elevator, Arm arm, DriveController driveController) {
    m_claw = claw;
    m_climb = climb;
    m_elevator = elevator;
    m_arm = arm;
    m_driveController = driveController;
  }

  public void setState(State state) {
    m_state = state;
  }

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

    if (!m_arm.motorAtTargetRotation()) {
      return false;
    }

    if (!m_elevator.motorAtTarget()) {
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

    m_claw.log();
    m_climb.log();
    m_elevator.log();
    m_arm.log();
  }

  public void syncSensors() {
    m_claw.syncSensors();
    m_climb.syncSensors();
    m_elevator.syncSensors();
    m_arm.syncSensors();
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

  public void armivatorTargetReef() {
    double x = m_driveController.getDriveWithLimelight().getDistFromScoring();
    double y = m_targetReefLevel.getHeight() - ElevatorInfo.FLOOR_TO_ELEVATOR_ZERO_METERS;

    ArmivatorPose pose = ArmivatorPose.fromCoordinate(x, y, ARMIVATOR_CONFIG);

    m_arm.setTargetDeg(pose.getArmAngle());
    m_elevator.setTargetPostion(pose.getElevatorHeight() * Conversions.Distance.INCHES_PER_METER);

    if (pose.getTargetIsOutOfBounds()) {
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
        } else {
          armStow();
          elevatorStow();
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

              m_manualArmivator = true;
            } else {
              armStow();
              elevatorStow();

              m_manualArmivator = false;
            }
            break;
          case Approach:
            armivatorTargetReef();

            m_manualArmivator = true;
            break;
          case MoveToScoring:
            armivatorTargetReef();
            break;
          case Scoring:
            if (m_manualScore) {
              clawScore();
            }

            armivatorTargetReef();
            break;
          case MoveToBackOff:
            m_claw.setControl(Claw.ControlStatus.Off);

            armivatorTargetReef();
            break;
          case BackOff:
            armivatorTargetReef();

            m_manualArmivator = false;
            break;
        }
        break;
      case Zero:
        m_arm.setControlStatus(Arm.ControlStatus.Zero);
        m_elevator.setControlStatus(Elevator.ControlStatus.Zero);

        clawIntake();
        break;
      case Climb:
        m_claw.setControl(Claw.ControlStatus.Off);

        setTargetReefLevel(ReefLevel.Horizontal);

        armTargetReefLevel();
        elevatorTargetReefLevel();

        m_manualArmivator = true;
        break;
      case Off:
        m_claw.setControl(Claw.ControlStatus.Off);
        m_arm.setControlStatus(Arm.ControlStatus.Off);
        m_elevator.setControlStatus(Elevator.ControlStatus.Off);
        break;
    }

    m_lastState = m_state;

    m_claw.update();
    m_climb.update();
    m_elevator.update();
    m_arm.update();
  }

  public void reset() {
    m_claw.reset();
    m_climb.reset();
    m_elevator.reset();
    m_arm.reset();
  }
}
