package com.team973.frc2025.subsystems;

import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure implements Subsystem {
  private final Claw m_claw;
  private final Climb m_climb;
  private final Elevator m_elevator;
  private final Arm m_arm;
  private final Wrist m_wrist;
  private final DriveController m_driveController;

  private State m_state = State.Manual;
  private State m_lastState = State.Manual;

  private GamePiece m_gamePieceMode = GamePiece.Coral;

  private ReefLevel m_targetReefLevel = ReefLevel.L_1;

  private boolean m_manualScore = false;
  private boolean m_manualIntake = true;
  private boolean m_manualArmivator = false;

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
      DriveController driveController) {
    m_claw = claw;
    m_climb = climb;
    m_elevator = elevator;
    m_arm = arm;
    m_wrist = wrist;
    m_driveController = driveController;
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

    return getHasAlgae();
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

  private void wristTargetReefLevel() {
    m_wrist.setTargetDeg(m_wrist.getTargetDegFromLevel(m_targetReefLevel));
    m_wrist.setControlStatus(Wrist.ControlStatus.TargetPostion);
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

  public GamePiece getGamePieceMode() {
    return m_gamePieceMode;
  }

  public void toggleGamePieceMode() {
    if (m_gamePieceMode == GamePiece.Coral) {
      setGamePieceMode(GamePiece.Algae);
      setTargetReefLevel(
          getAlgaePresetFromReefFace(
              m_driveController.getDriveWithLimelight().getTargetReefFace()));
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

  private ReefLevel getAlgaePresetFromReefFace(ReefFace face) {
    return switch (face) {
      case A, C, E -> ReefLevel.AlgaeHigh;
      case B, D, F -> ReefLevel.AlgaeLow;
      default -> throw new IllegalArgumentException(face.toString());
    };
  }

  public ReefLevel getTargetReefLevel() {
    return m_targetReefLevel;
  }

  public boolean getHasAlgae() {
    return m_claw.getHasAlgae();
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
            if (m_manualScore && m_gamePieceMode == GamePiece.Coral) {
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
            armStow();
            elevatorStow();
            wristStow();

            m_manualArmivator = false;
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
