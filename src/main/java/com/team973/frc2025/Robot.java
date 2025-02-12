// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team973.frc2025;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.BlinkingSignaler;
import com.team973.frc2025.subsystems.CANdleManger;
import com.team973.frc2025.subsystems.Arm;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.Claw.ControlStatus;
import com.team973.frc2025.subsystems.Climb;
import com.team973.frc2025.subsystems.Conveyor;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.SolidSignaler;
import com.team973.frc2025.subsystems.DriveController.ControllerOption;
import com.team973.frc2025.subsystems.Elevator;
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

  private final Climb m_climb = new Climb(m_logger.subLogger("climb manager"));

  private final Conveyor m_conveyor = new Conveyor(m_logger.subLogger("conveyor manager"));

  private final DriveController m_driveController =
      new DriveController(m_logger.subLogger("drive", 0.05));
  private final Claw m_claw = new Claw(m_logger.subLogger("claw", 0.2));

  private final Elevator m_elevator = new Elevator(m_logger.subLogger("elevator"));
  private final Arm m_arm = new Arm(m_logger.subLogger("Arm", 0.2));

  private final CANdleManger m_CaNdleManger = new CANdleManger(new Logger("candle manger"));
  public final BlinkingSignaler m_redBlinker =
      new BlinkingSignaler(
          new Logger("RedBlinking Signaler"),
          RobotInfo.Colors.RED,
          RobotInfo.Colors.GREEN,
          500,
          100);
  public final BlinkingSignaler m_blueBlinker =
      new BlinkingSignaler(
          new Logger("BlueBlinking Signaler"),
          RobotInfo.Colors.BLUE,
          RobotInfo.Colors.OFF,
          1000,
          0);
  public final SolidSignaler m_offBlinker = new SolidSignaler(RobotInfo.Colors.OFF, 90);
  private final AutoManager m_autoManager =
      new AutoManager(m_logger.subLogger("auto"), m_driveController, m_claw);

  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.SickStick, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));

  private void syncSensors() {
    m_driveController.syncSensors();
    m_claw.syncSensors();
    m_CaNdleManger.syncSensors();
    m_climb.syncSensors();
    m_conveyor.syncSensors();
  }

  private void updateSubsystems() {
    m_driveController.update();
    m_climb.update();
    m_conveyor.update();
    m_claw.update();
    m_CaNdleManger.update();
    m_elevator.update();
    m_arm.update();
  }

  private void resetSubsystems() {
    m_driveController.reset();
    m_claw.reset();
    m_CaNdleManger.reset();
  }

  private void logSubsystems() {
    // m_driveController.log();
    // m_claw.log();
    // m_CaNdleManger.log();
    m_climb.reset();
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
    m_offBlinker.setEnabled(true);
    m_CaNdleManger.addSignaler(m_blueBlinker);
    m_CaNdleManger.addSignaler(m_redBlinker);
    m_CaNdleManger.addSignaler(m_offBlinker);
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
    if (m_coDriverStick.getRightBumperButtonPressed()) {
      m_redBlinker.setEnabled(true);
    }
    if (m_coDriverStick.getRightBumperButtonReleased()) {
      m_redBlinker.setEnabled(false);
    }
    if (m_coDriverStick.getLeftBumperButtonPressed()) {
      m_blueBlinker.setEnabled(true);
    }
    if (m_coDriverStick.getLeftBumperButtonReleased()) {
      m_blueBlinker.setEnabled(false);
    }
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

    if (m_stick.getAButtonPressed()) {
      m_climb.setControlMode(Climb.ControlMode.ClimbHigh);
    } else if (m_stick.getBButtonPressed()) {
      m_climb.setControlMode(Climb.ControlMode.ClimbLow);
    } else if (m_stick.getXButtonPressed()) {
      m_climb.setControlMode(Climb.ControlMode.OffState);
    } else if (m_stick.getYButton()) {
      m_climb.setControlMode(Climb.ControlMode.JoystickMode);
      m_climb.setManualPower(m_stick.getLeftYAxis());
    } else if (m_stick.getYButtonReleased()) {
      m_climb.setControlMode(Climb.ControlMode.OffState);
    }

    if (m_teststick.getAButtonPressed()) {
      m_conveyor.setControlMode(Conveyor.ControlMode.ConveyorForward);
    } else if (m_teststick.getBButtonPressed()) {
      m_conveyor.setControlMode((Conveyor.ControlMode.ConveyorBackward));
    } else if (m_teststick.getXButtonPressed()) {
      m_conveyor.setControlMode((Conveyor.ControlMode.ConveyorOff));
    }

    if (m_coDriverStick.getAButton()) {
      m_claw.setControl(Claw.ControlStatus.IntakeAndHold);
    } else if (m_coDriverStick.getBButton()) {
      m_claw.setControl(Claw.ControlStatus.Stop);
    } else if (m_coDriverStick.getXButton()) {
      m_claw.setControl(Claw.ControlStatus.Score);
    } else if (m_coDriverStick.getYButton()) {
      m_claw.setControl(Claw.ControlStatus.Retract);
    }
    // if (m_coDriverStick.getPOVTop()) {
    //   m_elevator.setTargetPostion(Elevator.Presets.LEVEL_4);
    // } else if (m_coDriverStick.getPOVLeft()) {
    //   m_elevator.setTargetPostion(Elevator.Presets.LEVEL_3);
    // } else if (m_coDriverStick.getPOVRight()) {
    //   m_elevator.setTargetPostion(Elevator.Presets.LEVEL_2);
    // } else if (m_coDriverStick.getPOVBottom()) {
    //   m_elevator.setTargetPostion(Elevator.Presets.LEVEL_1);
    // } else if (m_coDriverStick.getLeftBumperButton()) {
    //   m_elevator.setmotorManualOutput(m_coDriverStick.getLeftYAxis());
    // } else {
    //   m_elevator.setModeOff();
    // }

    if (m_coDriverStick.getLeftBumperButton()) {
      m_arm.setArmTargetDeg(Arm.HIGH_POSTION_DEG);
    } else if (m_coDriverStick.getLeftTrigger()) {
      m_arm.setArmTargetDeg(Arm.LOW_POSTION_DEG);
    } else if (m_coDriverStick.getStartButton()) {
      m_arm.setArmTargetDeg(Arm.MEDIUM_POSTION_DEG);
    } else if (m_coDriverStick.getRightBumperButton()) {
      m_arm.setArmMotorManualOutput(m_coDriverStick.getRightYAxis() * -1.0);
    } else {
      m_arm.setStow();
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
    m_CaNdleManger.update();

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
