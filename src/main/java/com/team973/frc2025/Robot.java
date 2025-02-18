package com.team973.frc2025;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Arm;
import com.team973.frc2025.subsystems.CANdleManger;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.Climb;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.Elevator;
import com.team973.frc2025.subsystems.SolidSignaler;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight;
import com.team973.lib.util.Joystick;
import com.team973.lib.util.JoystickField;
import com.team973.lib.util.Logger;
import com.team973.lib.util.PerfLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final Logger m_logger = new Logger("robot");
  private final DriveController m_driveController =
      new DriveController(m_logger.subLogger("drive", 0.05));
  private final Climb m_climb = new Climb(m_logger.subLogger("climb manager"));
  private final CANdleManger m_candleManger = new CANdleManger(new Logger("candle manger"));
  private final Claw m_claw = new Claw(m_logger.subLogger("claw", 0.2), m_candleManger);
  private final Elevator m_elevator = new Elevator(m_logger.subLogger("elevator"));
  private final Arm m_arm = new Arm(m_logger.subLogger("Arm"));

  private final SolidSignaler m_lowBatterySignaler =
      new SolidSignaler(RobotInfo.Colors.ORANGE, 0, 1);

  private final SolidSignaler m_ledOff = new SolidSignaler(RobotInfo.Colors.OFF, 0, 100);
  private final Superstructure m_superstructure =
      new Superstructure(m_claw, m_climb, m_elevator, m_arm, m_driveController);

  private final AutoManager m_autoManager =
      new AutoManager(m_logger.subLogger("auto"), m_driveController, m_claw);
  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.SickStick, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));

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
      m_sideSelector.range(Rotation2d.fromDegrees(240 - 90), Rotation2d.fromDegrees(60), 0.5);
  private final JoystickField.Range m_rightReefSide =
      m_sideSelector.range(Rotation2d.fromDegrees(120 - 90), Rotation2d.fromDegrees(60), 0.5);

  private void syncSensors() {
    double startTime = Timer.getFPGATimestamp();
    m_driveController.syncSensors();

    m_superstructure.syncSensors();

    m_syncSensorsLogger.observe(Timer.getFPGATimestamp() - startTime);
  }

  private void updateSubsystems() {
    m_driveController.update();
    m_climb.update();
    m_claw.update();
    m_candleManger.update();
    m_elevator.update();
    m_arm.update();
    m_candleManger.update();
    m_superstructure.update();
  }

  private void resetSubsystems() {
    m_driveController.reset();
    m_superstructure.reset();
    m_claw.reset();
    m_candleManger.reset();
  }

  private void logSubsystems() {
    m_driveController.log();
    m_superstructure.log();
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
    m_ledOff.enable();
    m_candleManger.addSignaler(m_lowBatterySignaler);
    m_candleManger.addSignaler(m_ledOff);
    m_candleManger.addSignaler(m_claw.m_coralInClawSignaler);
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
    double startTime = Timer.getFPGATimestamp();
    logSubsystems();

    updateJoysticks();

    m_robotPeriodicLogger.observe(Timer.getFPGATimestamp() - startTime);
  }

  private void setPoseFromAuto() {
    m_driveController.resetOdometry(m_autoManager.getSelectedMode().getStartingPose(m_alliance));
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
    m_autoManager.init();
    m_driveController.resetOdometry(m_autoManager.getStartingPose(m_alliance));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    syncSensors();
    // TODO: we're doing this badly to make it work
    m_driveController.getDriveWithJoysticks().updateInput(0.0, 0.0, 0.0);
    // TODO: Figure out why autos don't work if updateSubsystems() comes before automanager.run().
    m_autoManager.run();
    updateSubsystems();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_driveController.setControllerOption(DriveController.ControllerOption.DriveWithJoysticks);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    syncSensors();

    maybeUpdateScoringSelection();

    double allianceScalar = 1.0;
    if (m_alliance == Alliance.Red) {
      // Our gyroscope is blue-centric meaning that facing away from the alliance wall
      // is a 0 degree heading. But the driver station is facing 180 when we are on the
      // red alliance. So when we are the red alliance we need to flip the joystick inputs.
      // Ideally we would convert this to polar coordinates, rotate by 180, and then convert
      // back to cartesian. But the algebra here is equivalent to just negating the X and Y
      // so that's what we iwll do for now.
      allianceScalar = -1.0;
    }
    m_driveController
        .getDriveWithJoysticks()
        .updateInput(
            allianceScalar * m_driverStick.getLeftXAxis() * 0.6,
            allianceScalar * m_driverStick.getLeftYAxis() * 0.6,
            m_driverStick.getRightXAxis() * 0.8);

    if (!m_driverStick.getRightTrigger()) {
      m_driveController.setControllerOption(ControllerOption.DriveWithJoysticks);
      m_superstructure.setState(Superstructure.State.Manual);

      if (m_driverStick.getRightBumperButtonPressed()) {
        m_superstructure.setManualScore(true);
      } else if (m_driverStick.getRightBumperButtonReleased()) {
        m_superstructure.setManualScore(false);
      }

      if (m_driverStick.getLeftBumperButtonPressed()) {
        m_superstructure.setManualArmivator(true);
      }

      if (m_driverStick.getLeftTriggerPressed()) {
        m_superstructure.setManualArmivator(false);
      }
    } else {
      m_driveController.setControllerOption(ControllerOption.DriveWithLimelight);
      m_driveController
          .getDriveWithLimelight()
          .targetReefPosition(
              () -> false,
              // () -> m_superstructure.readyToScore(),
              () -> m_superstructure.finishedScoring());
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
      m_superstructure.setTargetReefLevel(1);
    } else if (m_coDriverStick.getXButtonPressed()) {
      m_superstructure.setTargetReefLevel(2);
    } else if (m_coDriverStick.getBButtonPressed()) {
      m_superstructure.setTargetReefLevel(3);
    } else if (m_coDriverStick.getYButtonPressed()) {
      m_superstructure.setTargetReefLevel(4);
    }

    if ((climbStick) > 0.8) {
      m_superstructure.setState(Superstructure.State.ClimbLow);
    } else if ((climbStick) < -0.8) {
      m_superstructure.setState(Superstructure.State.ClimbStow);
    } else {
      m_superstructure.setClimbPower(0);
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
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  private PerfLogger m_disabledPeriodicLogger =
      new PerfLogger(m_logger.subLogger("perf/disabledPeriodic", 0.25));

  private void maybeUpdateScoringSelection() {
    if (m_frontFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(1);
    } else if (m_frontRightFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(2);
    } else if (m_backRightFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(3);
    } else if (m_backFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(4);
    } else if (m_backLeftFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(5);
    } else if (m_frontLeftFace.isActive()) {
      m_driveController.getDriveWithLimelight().setTargetReefFace(6);
    }

    if (m_driverStick.getRightTrigger()) {
      if (m_leftReefSide.isActive()) {
        m_driveController
            .getDriveWithLimelight()
            .setTargetSide(DriveWithLimelight.TargetReefSide.Left);
      } else if (m_rightReefSide.isActive()) {
        m_driveController
            .getDriveWithLimelight()
            .setTargetSide(DriveWithLimelight.TargetReefSide.Right);
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    double startTime = Timer.getFPGATimestamp();
    maybeInitAlliance();
    syncSensors();
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
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
