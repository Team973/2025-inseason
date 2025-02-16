package com.team973.frc2025;

import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.subsystems.Arm;
import com.team973.frc2025.subsystems.CANdleManger;
import com.team973.frc2025.subsystems.Claw;
import com.team973.frc2025.subsystems.Climb;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Elevator;
import com.team973.frc2025.subsystems.SolidSignaler;
import com.team973.frc2025.subsystems.Superstructure;
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
  private final Climb m_climb = new Climb(m_logger.subLogger("climb manager"));
  private final CANdleManger m_candleManger = new CANdleManger(new Logger("candle manger"));
  private final Claw m_claw = new Claw(m_logger.subLogger("claw", 0.2), m_candleManger);
  private final Elevator m_elevator = new Elevator(m_logger.subLogger("elevator"));
  private final Arm m_arm = new Arm(m_logger.subLogger("Arm", 0.2));

  private final SolidSignaler m_lowBatterySignaler =
      new SolidSignaler(RobotInfo.Colors.ORANGE, 0, 1);
  private final SolidSignaler m_coralInclawBlinker =
      new SolidSignaler(RobotInfo.Colors.GREEN, 2000, 2);

  private final SolidSignaler m_ledOff = new SolidSignaler(RobotInfo.Colors.OFF, 0, 100);
  private final Superstructure m_superstructure =
      new Superstructure(m_claw, m_climb, m_elevator, m_arm, m_driveController);

  private final AutoManager m_autoManager =
      new AutoManager(m_logger.subLogger("auto"), m_driveController, m_claw);
  private final Joystick m_driverStick =
      new Joystick(0, Joystick.Type.SickStick, m_logger.subLogger("driverStick"));
  private final Joystick m_coDriverStick =
      new Joystick(1, Joystick.Type.XboxController, m_logger.subLogger("coDriverStick"));
  private boolean m_manualScoringMode = true;

  private void syncSensors() {
    m_driveController.syncSensors();
    m_claw.syncSensors();
    m_candleManger.syncSensors();
    m_climb.syncSensors();

    m_superstructure.syncSensors();
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

    // SmartDashboard.putString("DB/String 3", "Manual: " + m_manualScoringMode);

    m_claw.log();
    m_candleManger.log();
    m_climb.reset();
    m_superstructure.reset();
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
    m_ledOff.setEnabled(true);
    m_candleManger.addSignaler(m_lowBatterySignaler);
    m_candleManger.addSignaler(m_ledOff);
    m_candleManger.addSignaler(m_coralInclawBlinker);
    m_candleManger.addSignaler(m_claw.m_coralInclawBlinker);
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

  public void teleopPeriodic() {

    double climbStick = m_coDriverStick.getLeftYAxis();

    if (m_coDriverStick.getAButton()) {
      if (m_coDriverStick.getPOVRightPressed()) {
        m_superstructure.incrementArmOffset(1.0);
      } else if (m_coDriverStick.getPOVLeftPressed()) {
        m_superstructure.incrementArmOffset(-1.0);
      }
    } else if (m_coDriverStick.getBButton()) {
      if (m_coDriverStick.getPOVRightPressed()) {
        m_superstructure.incrementElevatorOffset(0.5);
      } else if (m_coDriverStick.getPOVLeftPressed()) {
        m_superstructure.incrementElevatorOffset(-0.5);
      }
    } else if (m_coDriverStick.getYButton()) {
      if (m_coDriverStick.getPOVRightPressed()) {
        m_superstructure.incrementCoralBackup(0.5);
      } else if (m_coDriverStick.getPOVLeftPressed()) {
        m_superstructure.incrementCoralBackup(-0.5);
      }
    } else if ((climbStick) > 0.8) {
      m_superstructure.setState(Superstructure.State.ClimbLow);
    } else if ((climbStick) < -0.8) {
      m_superstructure.setState(Superstructure.State.ClimbStow);
    } else {
      m_superstructure.setClimbPower(0);
    }

    if (m_coDriverStick.getPOVTopPressed()) {
      m_superstructure.incrementTargetReefLevel(1);
    } else if (m_coDriverStick.getPOVBottomPressed()) {
      m_superstructure.incrementTargetReefLevel(-1);
    }
    // else if (m_coDriverStick.getPOVRightPressed()) {
    //   m_driveController.getDriveWithLimelight().incrementTargetReefFace(1);
    // } else if (m_coDriverStick.getPOVLeftPressed()) {
    //   m_driveController.getDriveWithLimelight().incrementTargetReefFace(-1);
    // }
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
        "DB/String 5", String.valueOf(m_autoManager.getSelectedMode().getName()));
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
