package com.team973.frc2025;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import choreo.util.ChoreoAllianceFlipUtil;
import com.team973.frc2025.shared.CrashTracker;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Arm;
import com.team973.frc2025.subsystems.BlinkingSignaler;
import com.team973.frc2025.subsystems.CANdleManger;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.Climb;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.Elevator;
import com.team973.frc2025.subsystems.SolidSignaler;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.frc2025.subsystems.Wrist;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight.ReefFace;
import com.team973.lib.util.Conversions;
import com.team973.lib.util.Joystick;
import com.team973.lib.util.JoystickField;
import com.team973.lib.util.Logger;
import com.team973.lib.util.PerfLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final AtomicBoolean m_readyToScore = new AtomicBoolean(false);
  private final AtomicBoolean m_readyToBackOff = new AtomicBoolean(false);

  private final Logger m_logger = new Logger("robot");
  private final DriveController m_driveController =
      new DriveController(m_logger.subLogger("drive", 0.05), m_readyToScore, m_readyToBackOff);
  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.SickStick, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));
  private final CANdleManger m_candleManger = new CANdleManger(new Logger("candle manger"));
  private final Climb m_climb = new Climb(m_logger.subLogger("climb manager"), m_candleManger);
  private final Claw m_claw = new Claw(m_logger.subLogger("claw", 0.2), m_candleManger);
  private final Elevator m_elevator = new Elevator(m_logger.subLogger("elevator"), m_candleManger);
  private final Arm m_arm = new Arm(m_logger.subLogger("Arm"), m_candleManger);
  private final Wrist m_wrist = new Wrist(m_logger.subLogger("wrist"));
  private final SolidSignaler m_lowBatterySignaler =
      new SolidSignaler(
          RobotInfo.Colors.ORANGE, 3000, RobotInfo.SignalerInfo.LOW_BATTER_SIGNALER_PRIORTY);

  private final SolidSignaler m_ledOff =
      new SolidSignaler(RobotInfo.Colors.OFF, 0, RobotInfo.SignalerInfo.OFF_SIGNALER_PRIORTY);

  private final BlinkingSignaler m_crashSignaler =
      new BlinkingSignaler(
          RobotInfo.Colors.RED,
          RobotInfo.Colors.OFF,
          200,
          1000,
          RobotInfo.SignalerInfo.CRASH_SIGNALER_PRIORITY);

  private final Superstructure m_superstructure =
      new Superstructure(m_claw, m_climb, m_elevator, m_arm, m_wrist, m_driveController);

  private final AutoManager m_autoManager =
      new AutoManager(m_logger.subLogger("auto"), m_driveController, m_superstructure);

  private PerfLogger m_syncSensorsLogger =
      new PerfLogger(m_logger.subLogger("perf/syncSensors", 0.25));

  private final JoystickField m_reefSelector =
      new JoystickField(
          () -> m_coDriverStick.getRightXAxis(), () -> m_coDriverStick.getRightYAxis());

  private final JoystickField.Range m_backFace =
      m_reefSelector.range(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(30), 0.8);
  private final JoystickField.Range m_backRightFace =
      m_reefSelector.range(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(30), 0.8);
  private final JoystickField.Range m_frontRightFace =
      m_reefSelector.range(Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(30), 0.8);
  private final JoystickField.Range m_frontFace =
      m_reefSelector.range(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(30), 0.8);
  private final JoystickField.Range m_frontLeftFace =
      m_reefSelector.range(Rotation2d.fromDegrees(240), Rotation2d.fromDegrees(30), 0.8);
  private final JoystickField.Range m_backLeftFace =
      m_reefSelector.range(Rotation2d.fromDegrees(300), Rotation2d.fromDegrees(30), 0.8);

  private final JoystickField m_sideSelector =
      new JoystickField(() -> m_driverStick.getLeftXAxis(), () -> m_driverStick.getLeftYAxis());
  private final JoystickField.Range m_leftReefSide =
      m_sideSelector.range(Rotation2d.fromDegrees(270), Rotation2d.fromDegrees(90), 0.5);
  private final JoystickField.Range m_rightReefSide =
      m_sideSelector.range(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), 0.5);

  private double m_lastBatteryVoltageHighMSTimestamp;
  private final double m_lowBatteryTimeOutMs = 1000.0;
  private final double m_lowBatterMimiumVoltage = 12.1;

  public static enum ControlStatus {
    HighBattery,
    LowBattery,
  }

  private void syncSensors() {
    double startTime = Timer.getFPGATimestamp();
    m_driveController.syncSensors();
    m_candleManger.syncSensors();
    m_superstructure.syncSensors();

    m_readyToScore.set(m_superstructure.readyToScore());
    m_readyToBackOff.set(m_superstructure.readyToBackOff());

    m_syncSensorsLogger.observe(Timer.getFPGATimestamp() - startTime);
  }

  private void updateSubsystems() {
    m_driveController.update();
    m_candleManger.update();
    m_superstructure.update();
  }

  private void resetSubsystems() {
    m_driveController.reset();
    m_superstructure.reset();
    m_candleManger.reset();
  }

  private void logSubsystems() {
    m_driveController.log();
    m_superstructure.log();
    m_candleManger.log();

    m_logger.log("Ready To Score", m_readyToScore.get());
    m_logger.log("Ready To Back Off", m_readyToBackOff.get());
  }

  private void updateJoysticks() {
    m_driverStick.update();
    m_coDriverStick.update();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    resetSubsystems();
    m_driveController.startOdometrey();
    m_candleManger.addSignaler(m_ledOff);
    m_candleManger.addSignaler(m_lowBatterySignaler);
    m_candleManger.addSignaler(m_crashSignaler);
    ChoreoAllianceFlipUtil.setYear(2025);
  }

  private PerfLogger m_robotPeriodicLogger =
      new PerfLogger(m_logger.subLogger("perf/robotPeriodic", 0.25));

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    try {
      m_ledOff.enable();

      if (RobotController.getBatteryVoltage() > m_lowBatterMimiumVoltage) {
        m_lastBatteryVoltageHighMSTimestamp = Conversions.Time.getMsecTime();
        m_lowBatterySignaler.disable();
      } else if (m_lastBatteryVoltageHighMSTimestamp + m_lowBatteryTimeOutMs
          < Conversions.Time.getMsecTime()) {
        m_lowBatterySignaler.enable();
      }
      if (CrashTracker.getExceptionHappened()) {
        m_crashSignaler.enable();
        CrashTracker.resetExceptionHappened();
      }
      double startTime = Timer.getFPGATimestamp();
      logSubsystems();

      updateJoysticks();

      m_robotPeriodicLogger.observe(Timer.getFPGATimestamp() - startTime);
    } catch (Exception e) {
      CrashTracker.logException("Robot Periodic", e);
    }
  }

  private void setPoseFromAuto() {
    m_driveController.resetOdometry(m_autoManager.getStartingPose(m_alliance));
  }

  private boolean m_allianceInitialized = false;
  private Alliance m_alliance;

  private void maybeInitAlliance() {
    if (m_allianceInitialized) {
      return;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (!alliance.isPresent()) {
      return;
    }
    m_allianceInitialized = true;
    m_alliance = alliance.get();
    setPoseFromAuto();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    try {
      m_autoManager.init();
      m_driveController.resetOdometry(m_autoManager.getStartingPose(m_alliance));
    } catch (Exception e) {
      CrashTracker.logException("Auto Init", e);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      syncSensors();
      // TODO: we're doing this badly to make it work
      m_driveController.getDriveWithJoysticks().updateInput(0.0, 0.0, 0.0);
      // TODO: Figure out why autos don't work if updateSubsystems() comes before
      // automanager.run().
      m_autoManager.run(m_alliance);
      updateSubsystems();
    } catch (Exception e) {
      CrashTracker.logException("Auto Periodic", e);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    try {
      m_driveController.setControllerOption(DriveController.ControllerOption.DriveWithJoysticks);
    } catch (Exception e) {
      CrashTracker.logException("Teleop Init", e);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      syncSensors();

      maybeUpdateScoringSelection();

      double allianceScalar = 1.0;
      if (m_alliance == Alliance.Red) {
        // Our gyroscope is blue-centric meaning that facing away from the alliance wall
        // is a 0 degree heading. But the driver station is facing 180 when we are on
        // the
        // red alliance. So when we are the red alliance we need to flip the joystick
        // inputs.
        // Ideally we would convert this to polar coordinates, rotate by 180, and then
        // convert
        // back to cartesian. But the algebra here is equivalent to just negating the X
        // and Y
        // so that's what we iwll do for now.
        allianceScalar = -1.0;
      }
      m_driveController
          .getDriveWithJoysticks()
          .updateInput(
              -allianceScalar * m_driverStick.getLeftYAxis(),
              allianceScalar * m_driverStick.getLeftXAxis(),
              m_driverStick.getRightXAxis() * 0.8);

      if (m_driverStick.getLeftBumperButtonPressed()) {
        m_superstructure.setManualArmivator(true);
      }

      if (m_driverStick.getLeftTriggerPressed()) {
        m_superstructure.setManualArmivator(false);
      }

      if (m_driverStick.getRightBumperButtonPressed()) {
        m_superstructure.setManualScore(true);
      } else if (m_driverStick.getRightBumperButtonReleased()) {
        m_superstructure.setManualScore(false);
      }

      if (m_driverStick.getRightTrigger()) {
        m_driveController.setControllerOption(ControllerOption.DriveWithLimelight);
        m_driveController.getDriveWithLimelight().targetReefPosition();
        m_superstructure.setState(Superstructure.State.Score);
      } else {
        m_driveController.setControllerOption(ControllerOption.DriveWithJoysticks);

        if (m_coDriverStick.getBackButton()) {
          m_superstructure.setState(Superstructure.State.Zero);
        } else {
          m_superstructure.setState(Superstructure.State.Manual);
        }
      }
      double climbStick = m_coDriverStick.getLeftYAxis();

      if (m_coDriverStick.getPOVTopPressed()) {
        m_superstructure.incrementElevatorOffset(0.5);
      } else if (m_coDriverStick.getPOVBottomPressed()) {
        m_superstructure.incrementElevatorOffset(-0.5);
      } else if (m_coDriverStick.getPOVRightPressed()) {
        m_superstructure.incrementArmOffset(1.0);
      } else if (m_coDriverStick.getPOVLeftPressed()) {
        m_superstructure.incrementArmOffset(-1.0);
      }

      if (m_coDriverStick.getAButtonPressed()) {
        m_superstructure.setTargetReefLevel(ReefLevel.L_1, ReefLevel.AlgaeFloor);
      } else if (m_coDriverStick.getXButtonPressed()) {
        m_superstructure.setTargetReefLevel(ReefLevel.L_2, ReefLevel.AlgaeLow);
      } else if (m_coDriverStick.getBButtonPressed()) {
        m_superstructure.setTargetReefLevel(ReefLevel.L_3, ReefLevel.AlgaeHigh);
      } else if (m_coDriverStick.getYButtonPressed()) {
        m_superstructure.setTargetReefLevel(ReefLevel.L_4, ReefLevel.Net);
      }

      if (m_coDriverStick.getRightBumperButtonPressed()) {
        m_superstructure.toggleGamePieceMode();
      }

      if (m_coDriverStick.getLeftBumperButtonPressed()) {
        m_superstructure.setManualIntake(false);
      } else if (m_coDriverStick.getLeftBumperButtonReleased()) {
        m_superstructure.setManualIntake(true);
      }

      if (m_coDriverStick.getRightTriggerPressed()) {
        m_superstructure.setClimbTarget(Climb.HORIZONTAL_POSITION_DEG);
        m_superstructure.setState(Superstructure.State.Climb);
      } else if (m_coDriverStick.getLeftTriggerPressed()) {
        m_superstructure.setClimbTarget(Climb.CLIMB_POSITION_DEG);
      } else if (Math.abs(climbStick) > 0.25) {
        m_superstructure.incrementClimbTarget(climbStick);
      }

      if (m_coDriverStick.getStartButtonPressed()) {
        // The drivers will always just point the robot _away_ from them. They give
        // inputs in alliance-wall centric coordinates. The robot itself though is
        // tracking angle in blue-centric coordinates. So when on the red alliance,
        // their zero position is actually 180 in blue-centric coordinates.
        if (m_alliance == Alliance.Blue) {
          m_driveController.resetAngle(Rotation2d.fromDegrees(0));
        } else {
          m_driveController.resetAngle(Rotation2d.fromDegrees(180));
        }
      }
      updateSubsystems();
    } catch (Exception e) {
      CrashTracker.logException("Teleop Periodic", e);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  private PerfLogger m_disabledPeriodicLogger =
      new PerfLogger(m_logger.subLogger("perf/disabledPeriodic", 0.25));

  private void maybeUpdateScoringSelection() {
    if (m_frontFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(ReefFace.A);
    } else if (m_frontRightFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(ReefFace.B);
    } else if (m_backRightFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(ReefFace.C);
    } else if (m_backFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(ReefFace.D);
    } else if (m_backLeftFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(ReefFace.E);
    } else if (m_frontLeftFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(ReefFace.F);
    }

    if (m_superstructure.getGamePieceMode() == Superstructure.GamePiece.Algae) {
      m_driveController.getDriveWithLimelight().setTargetSide(DriveWithLimelight.ReefSide.Center);
    } else if (m_driverStick.getRightTrigger()) {
      if (m_leftReefSide.isActive()) {
        m_driveController.getDriveWithLimelight().setTargetSide(DriveWithLimelight.ReefSide.Left);
      } else if (m_rightReefSide.isActive()) {
        m_driveController.getDriveWithLimelight().setTargetSide(DriveWithLimelight.ReefSide.Right);
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    try {
      double startTime = Timer.getFPGATimestamp();
      maybeInitAlliance();
      syncSensors();
      m_candleManger.update();
      // TODO: we're doing this badly to make it work
      m_driveController.getDriveWithJoysticks().updateInput(0.0, 0.0, 0.0);

      maybeUpdateScoringSelection();

      if (m_coDriverStick.getAButtonPressed()) {
        m_autoManager.increment();
        setPoseFromAuto();
      } else if (m_coDriverStick.getBButtonPressed()) {
        m_autoManager.decrement();
        setPoseFromAuto();
      }
      SmartDashboard.putString(
          "DB/String 5", String.valueOf(m_autoManager.getSelectedMode().getName()));

      m_disabledPeriodicLogger.observe(Timer.getFPGATimestamp() - startTime);
    } catch (Exception e) {
      CrashTracker.logException("Disabled Periodic", e);
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
