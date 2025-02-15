// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team973.frc2025;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.Claw.ControlStatus;
import com.team973.frc2025.subsystems.Climb;
import com.team973.frc2025.subsystems.Conveyor;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight;
import com.team973.lib.util.Joystick;
import com.team973.lib.util.Logger;
import com.team973.lib.util.PerfLogger;
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

  private final Climb m_climb = new Climb(m_logger.subLogger("climb manager"));

  private final Conveyor m_conveyor = new Conveyor(m_logger.subLogger("conveyor manager", 0.2));

  private final DriveController m_driveController =
      new DriveController(m_logger.subLogger("drive", 0.05));
  private final Claw m_claw = new Claw(m_logger.subLogger("claw", 0.2));

  private final AutoManager m_autoManager =
      new AutoManager(m_logger.subLogger("auto"), m_driveController, m_claw);

  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.SickStick, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));

  private PerfLogger m_syncSensorsLogger =
      new PerfLogger(m_logger.subLogger("perf/syncSensors", 0.25));

  private void syncSensors() {
    double startTime = Timer.getFPGATimestamp();
    m_driveController.syncSensors();

    if (RobotInfo.BOT_VERSION == RobotInfo.BotVersion.W1W) {
      m_climb.syncSensors();
      m_conveyor.syncSensors();
    }

    m_syncSensorsLogger.observe(Timer.getFPGATimestamp() - startTime);
  }

  private void updateSubsystems() {
    m_driveController.update();
    if (RobotInfo.BOT_VERSION == RobotInfo.BotVersion.W1W) {
      m_climb.update();
      m_conveyor.update();
    }
    m_claw.update();
  }

  private void resetSubsystems() {
    m_driveController.reset();
    if (RobotInfo.BOT_VERSION == RobotInfo.BotVersion.W1W) {
      m_climb.reset();
    }
  }

  private void logSubsystems() {
    m_driveController.log();
    m_claw.log();
    m_logger.update();
    if (RobotInfo.BOT_VERSION == RobotInfo.BotVersion.W1W) {
      m_conveyor.log();
      m_climb.log();
    }
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
  }

  private PerfLogger m_robotPeriodicLogger =
      new PerfLogger(m_logger.subLogger("perf/robotPeriodic"));

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

    maybeInitAlliance();

    if (m_driverStick.getLeftBumperButtonPressed()) {
      m_driveController.setControllerOption(ControllerOption.DriveWithLimelight);
      m_driveController
          .getDriveWithLimelight()
          .targetReefPosition(
              DriveWithLimelight.TargetReefSide.Left, () -> !m_claw.sensorSeeCoral());
    } else if (m_driverStick.getRightBumperButtonPressed()) {
      m_driveController.setControllerOption(ControllerOption.DriveWithLimelight);
      m_driveController
          .getDriveWithLimelight()
          .targetReefPosition(DriveWithLimelight.TargetReefSide.Right, () -> true);
    } else if (m_driverStick.getLeftBumperButtonReleased()
        || m_driverStick.getRightBumperButtonReleased()) {
      m_driveController.setControllerOption(ControllerOption.DriveWithJoysticks);
    }

    if (m_coDriverStick.getPOVRightPressed()) {
      m_driveController.getDriveWithLimelight().incrementTargetReefFace(1);
    } else if (m_coDriverStick.getPOVLeftPressed()) {
      m_driveController.getDriveWithLimelight().incrementTargetReefFace(-1);
    }

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

    if (RobotInfo.BOT_VERSION == RobotInfo.BotVersion.W1W) {
      m_driveController
          .getDriveWithJoysticks()
          .updateInput(
              m_driverStick.getLeftXAxis() * 0.5,
              m_driverStick.getLeftYAxis() * 0.5,
              m_driverStick.getRightXAxis() * 0.4);
    } else {
      m_driveController
          .getDriveWithJoysticks()
          .updateInput(
              m_driverStick.getLeftYAxis() * 0.2,
              -m_driverStick.getLeftXAxis() * 0.2,
              m_driverStick.getRightXAxis() * 0.2);
    }

    if (m_coDriverStick.getAButton()) {
      m_claw.setControl(ControlStatus.IntakeAndHold);
    } else if (m_coDriverStick.getBButton()) {
      m_claw.setControl(ControlStatus.Stop);
    } else if (m_coDriverStick.getXButton()) {
      m_claw.setControl(ControlStatus.Score);
    } else if (m_coDriverStick.getYButton()) {
      m_claw.setControl(ControlStatus.Retract);
    }

    updateSubsystems();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  private PerfLogger m_disabledPeriodicLogger =
      new PerfLogger(m_logger.subLogger("perf/disabledPeriodic", 0.25));

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    double startTime = Timer.getFPGATimestamp();
    syncSensors();

    // TODO: we're doing this badly to make it work
    m_driveController.getDriveWithJoysticks().updateInput(0.0, 0.0, 0.0);

    if (m_coDriverStick.getAButtonPressed()) {
      m_autoManager.increment();
      setPoseFromAuto();
    } else if (m_coDriverStick.getBButtonPressed()) {
      m_autoManager.decrement();
      setPoseFromAuto();
    }
    SmartDashboard.putString(
        "DB/String 2", String.valueOf(m_autoManager.getSelectedMode().getName()));

    m_disabledPeriodicLogger.observe(Timer.getFPGATimestamp() - startTime);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
