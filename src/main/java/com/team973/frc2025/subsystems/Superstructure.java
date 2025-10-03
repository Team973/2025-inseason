package com.team973.frc2025.subsystems;

import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.Colors;
import com.team973.frc2025.shared.RobotInfo.SignalerInfo;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

public class Superstructure implements Subsystem {
  private final RobotInfo.ElevatorInfo m_elevatorInfo;
  private final RobotInfo.WristInfo m_wristInfo;
  private final Claw m_claw;
  private final Climb m_climb;
  private final Elevator m_elevator;
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final DriveController m_driveController;
  private final RobotInfo.ArmInfo m_armInfo;

  private final Logger m_logger;

  private final SolidSignaler m_algaeSignaler =
      new SolidSignaler(Colors.CYAN, 100, SignalerInfo.ALGAE_MODE_SIGNALER_PRIORITY);
  private final SolidSignaler m_wristAndArmHorizontalSignaler =
      new SolidSignaler(
          RobotInfo.Colors.YELLOW, 250, RobotInfo.SignalerInfo.WRIST_HORIZONTAL_SIGNLAER_PRIORITY);

  private State m_state = State.Manual;
  private State m_lastState = State.Manual;

  private GamePiece m_gamePieceMode = GamePiece.Coral;
  private Supplier<ReefLevel> m_targetReefLevelSupplier = () -> ReefLevel.L_1;

  private boolean m_manualScore = false;
  private boolean m_manualIntake = true;
  private boolean m_manualArmivator = false;
  private boolean m_pickedUpAlgaeFromFloor = false;

  public enum State {
    Manual,
    Score,
    Zero,
    Climb,
    Off
  }

  public enum ReefLevel {
    L_1,
    L_2,
    L_3,
    L_4,
    AlgaeHigh,
    AlgaeLow,
    AlgaeFloor,
    Net,
    Processor,
    Horizontal
  }

  public enum GamePiece {
    Coral,
    Algae
  }

  public Superstructure(
      Claw claw,
      Climb climb,
      Elevator elevator,
      Arm arm,
      Wrist wrist,
      DriveController driveController,
      Logger logger,
      CANdleManger candle) {
    m_claw = claw;
    m_climb = climb;
    m_elevator = elevator;
    m_arm = arm;
    m_wrist = wrist;
    m_driveController = driveController;
    m_elevatorInfo = RobotConfig.get().ELEVATOR_INFO;
    m_wristInfo = RobotConfig.get().WRIST_INFO;
    m_armInfo = RobotConfig.get().ARM_INFO;

    m_logger = logger;

    candle.addSignaler(m_algaeSignaler);
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

  private void setTargetReefLevelSupplier(Supplier<ReefLevel> reefLevelSupplier) {
    m_targetReefLevelSupplier = reefLevelSupplier;

    if (!getHasAlgae()) {
      m_pickedUpAlgaeFromFloor = false;
    }
  }

  public void setTargetReefLevel(ReefLevel level) {
    setTargetReefLevelSupplier(() -> level);
  }

  public void setTargetReefLevel(ReefLevel coralLevel, ReefLevel algaeLevel) {
    if (m_gamePieceMode == GamePiece.Coral) {
      setTargetReefLevel(coralLevel);
    } else {
      setTargetReefLevel(algaeLevel);
    }
  }

  public void setTargetReefLevel(
      ReefLevel coralLevel, ReefLevel waitingForAlgaeLevel, ReefLevel hasAlgaeLevel) {
    if (m_gamePieceMode == GamePiece.Coral) {
      setTargetReefLevel(coralLevel);
    } else {
      setTargetReefLevelSupplier(
          () -> m_pickedUpAlgaeFromFloor ? hasAlgaeLevel : waitingForAlgaeLevel);
    }
  }

  public boolean readyToScore() {
    if (m_arm.getTargetPosition() == m_armInfo.CORAL_STOW_POSITION_DEG) {
      return false;
    }

    if (m_arm.getTargetPosition() == m_armInfo.ALGAE_STOW_POSITION_DEG
        && m_targetReefLevelSupplier.get() != ReefLevel.Processor) {
      return false;
    }

    if (m_elevator.getTargetPosition() == m_elevatorInfo.CORAL_STOW) {
      return false;
    }

    if (m_elevator.getTargetPosition() == m_elevatorInfo.ALGAE_STOW
        && m_targetReefLevelSupplier.get() != ReefLevel.Processor) {
      return false;
    }

    if (m_wrist.getTargetPosition() == m_wristInfo.WITH_CORAL_STOW_POSTION_DEG
        || m_wrist.getTargetPosition() == m_wristInfo.WITHOUT_CORAL_STOW_POSITION_DEG) {
      return false;
    }

    if (m_wrist.getTargetPosition() == m_wristInfo.ALGAE_STOW_POSITION_DEG
        && m_targetReefLevelSupplier.get() != ReefLevel.Processor) {
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
    } else if (m_targetReefLevelSupplier.get() == ReefLevel.Processor
        || m_targetReefLevelSupplier.get() == ReefLevel.Net) {
      return !m_claw.getHasAlgae();
    }

    return m_claw.getHasAlgae();
  }

  public void log() {
    SmartDashboard.putString("DB/String 0", "Reef Level: " + m_targetReefLevelSupplier.get());
    SmartDashboard.putString(
        "DB/String 1",
        "E: "
            + String.valueOf(
                m_elevator.getTargetPositionFromLevel(m_targetReefLevelSupplier.get())));
    SmartDashboard.putString(
        "DB/String 2",
        "A: " + String.valueOf(m_arm.getTargetDegFromLevel(m_targetReefLevelSupplier.get())));
    SmartDashboard.putString("DB/String 8", m_gamePieceMode.toString());

    m_logger.log("Game Piece Mode", m_gamePieceMode.toString());
    m_logger.log("State", m_state.toString());

    m_logger.log("Manual Score", m_manualScore);
    m_logger.log("Manual Intake", m_manualIntake);
    m_logger.log("Manual Armivator", m_manualArmivator);
    m_logger.log("Picked Up Algae From Floor", m_pickedUpAlgaeFromFloor);

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

    if (m_gamePieceMode == GamePiece.Algae) {
      m_algaeSignaler.enable();
    } else {
      m_algaeSignaler.disable();
    }

    if (m_wrist.isHorizontal() && m_arm.isHorizontal()) {
      m_wristAndArmHorizontalSignaler.enable();
    } else {
      m_wristAndArmHorizontalSignaler.disable();
    }

    if (m_targetReefLevelSupplier.get() == ReefLevel.AlgaeFloor && getHasAlgae()) {
      m_pickedUpAlgaeFromFloor = true;
    }
  }

  private void armTargetReefLevel() {
    m_arm.setTargetDeg(m_arm.getTargetDegFromLevel(m_targetReefLevelSupplier.get()));
    m_arm.setControlStatus(Arm.ControlStatus.TargetPostion);
  }

  private void armStow() {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_arm.setTargetDeg(m_armInfo.CORAL_STOW_POSITION_DEG);
    } else {
      m_arm.setTargetDeg(m_armInfo.ALGAE_STOW_POSITION_DEG);
    }
    m_arm.setControlStatus(Arm.ControlStatus.TargetPostion);
  }

  private void elevatorTargetReefLevel() {
    m_elevator.setTargetPostion(
        m_elevator.getTargetPositionFromLevel(m_targetReefLevelSupplier.get()));
    m_elevator.setControlStatus(Elevator.ControlStatus.TargetPostion);
  }

  private void elevatorStow() {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_elevator.setTargetPostion(m_elevatorInfo.CORAL_STOW);
    } else {
      m_elevator.setTargetPostion(m_elevatorInfo.ALGAE_STOW);
    }
    m_elevator.setControlStatus(Elevator.ControlStatus.TargetPostion);
  }

  private void wristStow() {
    if (m_gamePieceMode == GamePiece.Coral) {
      m_wrist.setTargetDeg(m_wristInfo.WITHOUT_CORAL_STOW_POSITION_DEG);
    } else {
      m_wrist.setTargetDeg(m_wristInfo.ALGAE_STOW_POSITION_DEG);
    }
    m_wrist.setControlStatus(Wrist.ControlStatus.TargetPostion);
  }

  private void wristTargetReefLevel() {
    m_wrist.setTargetDeg(m_wrist.getTargetDegFromLevel(m_targetReefLevelSupplier.get()));
    m_wrist.setControlStatus(Wrist.ControlStatus.TargetPostion);
  }

  public void incrementArmOffset(double increment) {
    m_arm.incrementOffset(increment, m_targetReefLevelSupplier.get());
  }

  public void setManualArmivator(boolean manual) {
    m_manualArmivator = manual;
  }

  public void incrementElevatorOffset(double increment) {
    m_elevator.incrementOffset(increment, m_targetReefLevelSupplier.get());
  }

  public void setGamePieceMode(GamePiece gamePiece) {
    m_gamePieceMode = gamePiece;
  }

  public GamePiece getGamePieceMode() {
    return m_gamePieceMode;
  }

  public void toggleGamePieceMode() {
    if (m_gamePieceMode == GamePiece.Coral) {
      setGamePieceMode(GamePiece.Algae);
      if (m_driveController.getDriveWithLimelight().getTargetReefFace() != ReefFace.Net
          && m_driveController.getDriveWithLimelight().getTargetReefFace() != ReefFace.Processor) {
        setTargetReefLevel(
            getAlgaePresetFromReefFace(
                m_driveController.getDriveWithLimelight().getTargetReefFace()));
      }
    } else {
      setGamePieceMode(GamePiece.Coral);
      m_driveController
          .getDriveWithLimelight()
          .setTargetSide(m_driveController.getDriveWithLimelight().getLastTargetReefSide());
    }
  }

  public void incrementCoralBackup(double increment) {
    m_claw.incrementBackup(increment);
  }

  public void incrementClimbTarget(double increment) {
    m_climb.incrementTarget(increment);
  }

  public void setClimbTarget(double target) {
    m_climb.setTargetAngleDeg(target);
  }

  public void clawIntake() {
    if (!m_manualIntake) {
      m_claw.setControl(Claw.ControlStatus.Off);
    } else if (m_gamePieceMode == GamePiece.Coral) {
      m_claw.setControl(Claw.ControlStatus.IntakeCoral);
    } else if (m_targetReefLevelSupplier.get() == ReefLevel.AlgaeFloor) {
      m_claw.setControl(Claw.ControlStatus.IntakeAlgaeFromFloor);
    } else {
      m_claw.setControl(Claw.ControlStatus.IntakeAlgae);
    }
  }

  public void clawScore() {
    if (m_gamePieceMode == GamePiece.Coral) {
      if (m_targetReefLevelSupplier.get() == ReefLevel.L_1) {
        m_claw.setControl(Claw.ControlStatus.ScoreCoralLevelOne);
      } else {
        m_claw.setControl(Claw.ControlStatus.ScoreCoral);
      }
    } else {
      if (m_targetReefLevelSupplier.get() == ReefLevel.Processor) {
        m_claw.setControl(Claw.ControlStatus.ScoreAlgaeProcessor);
      } else {
        m_claw.setControl(Claw.ControlStatus.ScoreAlgae);
      }
    }
  }

  public boolean getHasAlgae() {
    return m_claw.getHasAlgae();
  }

  public ReefLevel getAlgaePresetFromReefFace(ReefFace face) {
    return switch (face) {
      case A, C, E -> ReefLevel.AlgaeHigh;
      case B, D, F -> ReefLevel.AlgaeLow;
      default -> throw new IllegalArgumentException(face.toString());
    };
  }

  public ReefLevel getTargetReefLevel() {
    return m_targetReefLevelSupplier.get();
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
          m_claw.setControl(Claw.ControlStatus.Reverse);
        }

        if (m_manualArmivator) {
          armTargetReefLevel();
          elevatorTargetReefLevel();
          wristTargetReefLevel();
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
            if (m_driveController.isNearApproach()) {
              armTargetReefLevel();
              wristTargetReefLevel();
              elevatorTargetReefLevel();

              m_manualArmivator = true;
            }
            break;
          case Approach:
            armTargetReefLevel();
            elevatorTargetReefLevel();
            wristTargetReefLevel();

            m_manualArmivator = true;
            break;
          case MoveToScoring:
            armTargetReefLevel();
            elevatorTargetReefLevel();
            wristTargetReefLevel();
            break;
          case Scoring:
            if (m_manualScore
                && (m_gamePieceMode == GamePiece.Coral
                    || m_targetReefLevelSupplier.get() == ReefLevel.Processor
                    || m_targetReefLevelSupplier.get() == ReefLevel.Net)) {
              clawScore();
            }

            armTargetReefLevel();
            elevatorTargetReefLevel();
            wristTargetReefLevel();
            break;
          case MoveToBackOff:
            if (m_gamePieceMode == GamePiece.Coral) {
              m_claw.setControl(Claw.ControlStatus.Off);
            } else {
              clawIntake();
            }

            armTargetReefLevel();
            elevatorTargetReefLevel();
            wristTargetReefLevel();
            break;
          case BackOff:
            if (m_gamePieceMode == GamePiece.Coral || DriverStation.isTeleop()) {
              armStow();
              elevatorStow();
              wristStow();
              m_manualArmivator = false;
            }

            break;
        }
        break;
      case Zero:
        m_elevator.setControlStatus(Elevator.ControlStatus.Zero);

        wristStow();
        armStow();
        clawIntake();
        break;
      case Climb:
        m_claw.setControl(Claw.ControlStatus.Off);

        setTargetReefLevel(ReefLevel.Horizontal);

        armTargetReefLevel();
        elevatorTargetReefLevel();
        wristTargetReefLevel();

        m_manualArmivator = true;
        break;
      case Off:
        m_claw.setControl(Claw.ControlStatus.Off);
        m_arm.setControlStatus(Arm.ControlStatus.Off);
        m_elevator.setControlStatus(Elevator.ControlStatus.Off);
        m_wrist.setControlStatus(Wrist.ControlStatus.Off);
        break;
    }

    if (m_lastState == State.Zero && m_state != State.Zero) {
      m_elevator.home();
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
