// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team973.frc2025;

import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.Climb;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.frc2025.subsystems.composables.DriveWithLimelight;
import com.team973.lib.util.Joystick;
import com.team973.lib.util.Logger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final Logger m_logger = new Logger("robot");

  private final Joystick m_stick;

  private final Joystick m_teststick;

  private final DriveController m_driveController =
      new DriveController(m_logger.subLogger("drive", 0.05));

  private final Climb m_climb = new Climb(m_logger.subLogger("climb manager"));
  private final Claw m_claw = new Claw(m_logger.subLogger("claw", 0.2));

  private final Superstructure m_superstructure =
      new Superstructure(m_claw, m_climb, m_driveController);

  private final AutoManager m_autoManager =
      new AutoManager(m_logger.subLogger("auto"), m_driveController, m_claw);

  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.SickStick, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));

  private void syncSensors() {
    m_driveController.syncSensors();
    m_superstructure.syncSensors();
  }

  private void updateSubsystems() {
    m_driveController.update();
    m_superstructure.update();
  }

  private void resetSubsystems() {
    m_driveController.reset();
    m_superstructure.reset();
  }

  private void logSubsystems() {
    m_driveController.log();
    m_superstructure.log();

    m_logger.update();
  }

  private void updateJoysticks() {
    m_driverStick.update();
    m_coDriverStick.update();
    m_stick.update();
    m_teststick.update();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    resetSubsystems();
    m_driveController.startOdometrey();
    m_stick = new Joystick(2, Joystick.Type.XboxController, m_logger.subLogger("sticks"));
    m_teststick = new Joystick(3, Joystick.Type.XboxController, m_logger.subLogger("sticks"));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    logSubsystems();
    updateJoysticks();
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
    m_driveController.resetOdometry(m_autoManager.getStartingPose());
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

    m_driveController
        .getDriveWithJoysticks()
        .updateInput(
            m_driverStick.getLeftXAxis() * 0.95,
            m_driverStick.getLeftYAxis() * 0.95,
            m_driverStick.getRightXAxis() * 0.8);

    if (m_driverStick.getLeftBumperButtonPressed()) {
      m_driveController.setControllerOption(ControllerOption.DriveWithLimelight);
      m_driveController
          .getDriveWithLimelight()
          .targetReefPosition(
              DriveWithLimelight.TargetReefSide.Left,
              () -> m_superstructure.readyToScore(),
              () -> m_superstructure.finishedScoring());
    } else if (m_driverStick.getRightBumperButtonPressed()) {
      m_driveController.setControllerOption(ControllerOption.DriveWithLimelight);
      m_driveController
          .getDriveWithLimelight()
          .targetReefPosition(
              DriveWithLimelight.TargetReefSide.Right,
              () -> m_superstructure.readyToScore(),
              () -> m_superstructure.finishedScoring());
    } else if (m_driverStick.getLeftBumperButtonReleased()
        || m_driverStick.getRightBumperButtonReleased()) {
      m_driveController.setControllerOption(ControllerOption.DriveWithJoysticks);
    }

    if (m_coDriverStick.getYButton()) {
      m_superstructure.setState(Superstructure.State.Manual);

      if (m_coDriverStick.getXButtonPressed()) {
        m_superstructure.setManualScore(true);
      } else if (m_coDriverStick.getXButtonReleased()) {
        m_superstructure.setManualScore(false);
      }
    } else if (m_coDriverStick.getXButton()) {
      m_superstructure.setState(Superstructure.State.ScoreCoral);
    } else if (m_coDriverStick.getBButton()) {
      m_superstructure.setState(Superstructure.State.Climb);
    } else {
      m_superstructure.setState(Superstructure.State.IntakeCoral);
    }

    if (m_coDriverStick.getPOVTopPressed()) {
      m_superstructure.incrementTargetReefLevel(1);
    } else if (m_coDriverStick.getPOVBottomPressed()) {
      m_superstructure.incrementTargetReefLevel(-1);
    } else if (m_coDriverStick.getPOVRightPressed()) {
      m_driveController.getDriveWithLimelight().incrementTargetReefFace(1);
    } else if (m_coDriverStick.getPOVLeftPressed()) {
      m_driveController.getDriveWithLimelight().incrementTargetReefFace(-1);
    }

    updateSubsystems();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    syncSensors();

    // TODO: we're doing this badly to make it work
    m_driveController.getDriveWithJoysticks().updateInput(0.0, 0.0, 0.0);

    if (m_coDriverStick.getAButtonPressed()) {
      m_autoManager.increment();
    } else if (m_coDriverStick.getBButtonPressed()) {
      m_autoManager.decrement();
    }
    SmartDashboard.putString(
        "DB/String 2", String.valueOf(m_autoManager.getSelectedMode().getName()));
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
