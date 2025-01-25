// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team973.frc2025;

import com.team973.frc2025.subsystems.Arm;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Elevator;
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

  private final DriveController m_driveController =
      new DriveController(m_logger.subLogger("drive", 0.05));
  private final Claw m_claw = new Claw(new Logger("claw", 0.2));

  private final Elevator m_elevator = new Elevator(new Logger("elevator"));
  private final Arm m_arm = new Arm(new Logger("Arm"));

  private final AutoManager m_autoManager =
      new AutoManager(m_logger.subLogger("auto"), m_driveController, m_claw);

  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.SickStick, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));

  private void syncSensors() {
    m_driveController.syncSensors();
  }

  private void updateSubsystems() {
    m_driveController.update();
    m_claw.update();
  }

  private void resetSubsystems() {
    m_driveController.reset();
  }

  private void logSubsystems() {
    m_driveController.log();
    m_claw.log();
    m_logger.update();
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

    if (m_coDriverStick.getAButton()) {
      m_claw.setControl(Claw.ControlStatus.IntakeAndHold);
    } else if (m_coDriverStick.getBButton()) {
      m_claw.setControl(Claw.ControlStatus.Stop);
    } else if (m_coDriverStick.getXButton()) {
      m_claw.setControl(Claw.ControlStatus.Score);
    } else if (m_coDriverStick.getYButton()) {
      m_claw.setControl(Claw.ControlStatus.Retract);
    }
    if (m_coDriverStick.getPOVTop()) {
      m_elevator.setTargetPostion(Elevator.Presets.LEVEL_4);
    } else if (m_coDriverStick.getPOVLeft()) {
      m_elevator.setTargetPostion(Elevator.Presets.LEVEL_3);
    } else if (m_coDriverStick.getPOVRight()) {
      m_elevator.setTargetPostion(Elevator.Presets.LEVEL_2);
    } else if (m_coDriverStick.getPOVBottom()) {
      m_elevator.setTargetPostion(Elevator.Presets.LEVEL_1);
    } else {
      m_elevator.setTargetPostion(Elevator.Presets.OFF);
    }

    if (m_coDriverStick.getLeftBumperButton()) {
      m_arm.setArmTargetDeg(Arm.HIGH_POSTION_DEG);
    } else if (m_coDriverStick.getLeftTrigger()) {
      m_arm.setArmTargetDeg(Arm.LOW_POSTION_DEG);
    } else if (m_coDriverStick.getStartButton()) {
      m_arm.setArmTargetDeg(Arm.MEDIUM_POSTION_DEG);
    } else {
      m_arm.setStow();
    }

    updateSubsystems();
    updateJoysticks();
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
