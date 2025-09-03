package com.team973.frc2025.shared;

import com.team973.lib.util.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Color;

/** Robot info, specs, dimensions. */
public class RobotInfo {

  public enum BotVersion {
    AlphaBot,
    W1W,
  }

  public static final BotVersion BOT_VERSION = BotVersion.W1W;

  public static double fromBotVersion(double alphaValue, double w1wvalue) {
    switch (BOT_VERSION) {
      case AlphaBot:
        return alphaValue;
      case W1W:
        return w1wvalue;
      default:
        throw new IllegalArgumentException("Unhandled bot version: " + BOT_VERSION);
    }
  }

  public static final String CANIVORE_CANBUS = "Canivore"; // "Canivore";
  public final String ROBORIO_CANBUS = "";

  public ArmInfo ARM_INFO = new ArmInfo();

  public ClimbInfo CLIMB_INFO = new ClimbInfo();

  public ClawInfo CLAW_INFO = new ClawInfo();

  public DriveInfo DRIVE_INFO = new DriveInfo();

  public static class ArmInfo {
    public int HALL_SENSOR_ID = 31;
    public int ENCODER_CAN_ID = 32;
    public double ARM_ROTATIONS_PER_MOTOR_ROTATIONS = (10.0 / 84.0) * (16.0 / 108.0);

    public double ENCODER_OFFSET_ROTATIONS = -0.1663;

    public double HORIZONTAL_POSITION_DEG = 0.0;

    public double LEVEL_FOUR_POSITION_DEG = 65.0;
    public double LEVEL_THREE_POSITION_DEG = 59.0;
    public double LEVEL_TWO_POSITION_DEG = -61.0;
    public double LEVEL_ONE_POSITION_DEG = -64.0;
    public double CORAL_STOW_POSITION_DEG = -88.0;

    public double NET_POSITION_DEG = 74.0;
    public double ALGAE_HIGH_POSITION_DEG = 57.5;
    public double ALGAE_LOW_POSITION_DEG = -56.0;
    public double ALGAE_FLOOR_POSITION_DEG = -53.0;
    public double ALGAE_STOW_POSITION_DEG = -85.0;

    public double CENTER_GRAVITY_OFFSET_DEG = 3;
    public double FEED_FORWARD_MAX_VOLT = 0.32;

    public double m_levelOneOffset = 0.0;
    public double m_levelTwoOffset = 0.0;
    public double m_levelThreeOffset = 0.0;
    public double m_levelFourOffset = 0.0;

    public double m_netOffset = 0.0;
    public double m_algaeHighOffset = 0.0;
    public double m_algaeLowOffset = 0.0;
    public double m_algaeFloorOffset = 0.0;

    public double ARM_KS = 0.0;
    public double ARM_KV = 0.0;
    public double ARM_KA = 0.0;
    public double ARM_KP = 2.0;
    public double ARM_KI = 0.0;
    public double ARM_KD = 0.0;

    public double ARM_MOTION_MAGIC_CRUISE_VELOCITY = 110.0;
    public double ARM_MOTION_MAGIC_ACCELERATION = 230.0;
    public double ARM_MOTION_MAGIC_JERK = 0.0;

    public double ARM_SATOR_CURRENT_LIMIT = 60.0;
    public boolean ARM_SATOR_CURRENT_LIMIT_ENABLE = true;
    public double ARM_SUPPLY_CURRENT_LIMIT = 30.0;
    public boolean ARM_SUPPLY_CURRENT_LIMIT_ENABLE = true;

    public double ARM_PEAK_FORDWARD_VOLTAGE = 12.0;
    public double ARM_PEAK_REVERSE_VOLTAGE = -12.0;
  }

  public class ElevatorInfo {
    public int HALL_SENSOR_ID = 0;
  }

  public class ClawInfo {
    public int RIGHT_MOTOR_ID = 36;
    public int LEFT_MOTOR_ID = 35;
    public int CONVEYOR_MOTOR_ID = 34;

    public int CONVEYOR_BACK_SENSOR_ID = 2;
    public int CONVEYOR_FRONT_SENSOR_ID = 3;
    public int CLAW_ALGAE_CAN_ID = 48;

    public double ALGAE_SENSOR_STRENGTH_THRESHOLD = 2500.0;

    public double CONVEYOR_KP = 0.3;
    public double CONVEYOR_KV = 0.1;

    public double CLAW_KS = 0.0;
    public double CLAW_KV = 0.125 * 10.0 / 10.5;
    public double CLAW_KA = 0.0;
    public double CLAW_KP = 0.3;
    public double CLAW_KI = 0.0;
    public double CLAW_KD = 0.0;

    public double CLAW_SATOR_CURRENT_LIMIT = 60.0;
    public boolean CLAW_SATOR_CURRENT_LIMIT_ENABLE = true;
    public double CLAW_SUPPLY_CURRENT_LIMIT = 30.0;
    public boolean CLAW_SUPPLY_CURRENT_LIMIT_ENABLE = true;

    public double CLAW_PEAK_FORDWARD_VOLTAGE = 12.0;
    public double CLAW_PEAK_REVERSE_VOLTAGE = -12.0;

    public double CLAW_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.02;
  }

  public class WristInfo {
    public int MOTOR_CAN_ID = 31;
    public int ENCODER_CAN_ID = 35;
    public double ENCODER_OFFSET_ROTATIONS = -0.0596 + 0.25;
    public double WRIST_ROTATIONS_PER_MOTOR_ROTATIONS = 10.0 / 46.0 * 14.0 / 72.0 * 34.0 / 60.0;
  }

  public class ClimbInfo {
    public double MOTOR_ROT_PER_CLIMB_ROT =
        1.0 / ((10.0 / 64.0) * (20.0 / 72.0) * (20.0 / 84.0) * (9.0 / 30.0));

    public double JOYSTICK_TO_MOTOR_ROTATIONS = 5.0;
    public double MOTION_MAGIC_CRUISE_VELOCITY = JOYSTICK_TO_MOTOR_ROTATIONS * 20.0;

    public double HORIZONTAL_POSITION_DEG = 100.0;
    public double CLIMB_POSITION_DEG = 263.0;

    public double STOP_POSITION_DEG = 263.0;
  }

  public static class SignalerInfo {
    public static final int CRASH_SIGNALER_PRIORITY = 5;
    public static final int CLIMB_STOP_PRIORITY = 6;
    public static final int CLIMB_HORIZONTAL_PRIORITY = 7;
    public static final int ELEVATOR_HALL_SENSOR_SIGNALER_PRIORITY = 10;
    public static final int PEICE_IN_CLAW_SIGNALER_PRIORTY = 50;
    public static final int ALGAE_MODE_SIGNALER_PRIORITY = 60;
    public static final int ARM_HORIZONTAL_SIGNALER_PRIORTY = 97;
    public static final int WRIST_HORIZONTAL_SIGNLAER_PRIORITY = 98;
    public static final int LOW_BATTER_SIGNALER_PRIORTY = 99;
    public static final int OFF_SIGNALER_PRIORTY = 100;
  }

  public static class DriveInfo {
    public int STATUS_SIGNAL_FREQUENCY = 200;

    public static int PIGEON_ID = 1;

    public int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
    public static double FRONT_LEFT_MODULE_STEER_OFFSET = fromBotVersion(-743.027, 264.902);

    public int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
    public int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static double FRONT_RIGHT_MODULE_STEER_OFFSET = fromBotVersion(-684.931, 153.721);

    public int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public int BACK_LEFT_MODULE_STEER_ENCODER = 7;
    public static double BACK_LEFT_MODULE_STEER_OFFSET = fromBotVersion(-1249.628, 50.713);

    public int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    public static double BACK_RIGHT_MODULE_STEER_OFFSET = fromBotVersion(-490.517, 100.107);

    public double DRIVE_GEAR_RATIO = (10.0 / 54.0) * (40.0 / 16.0) * (15.0 / 45.0); // x3:10, 6.48:1

    public double ANGLE_GEAR_RATIO = (10.0 / 22.0) * (16.0 / 88.0); // 12.1:1

    public double WHEEL_DIAMETER_METERS = 0.1016;
    public double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    /**
     * The left-to-right distance between the drivetrain wheels Should be measured from center to
     * center.
     */
    public static double TRACKWIDTH_METERS = fromBotVersion(0.635, 0.5334);

    /**
     * The front-to-back distance between the drivetrain wheels. Should be measured from center to
     * center.
     */
    public static double WHEELBASE_METERS = fromBotVersion(0.635, 0.5334);

    public double OPEN_LOOP_RAMP = 0.0;
    public double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public double ANGLE_KP = 5.5; // 6.5;
    public double ANGLE_KI = 0.0;
    public double ANGLE_KD = 0.0;
    public double ANGLE_KF = 0.0;
    public double ANGLE_KV = 0.0;

    /* Drive Motor PID Values */
    public double DRIVE_KP = 0.38;
    public double DRIVE_KI = 0.0;
    public double DRIVE_KD = 0.0;
    public double DRIVE_KF = 0.12;

    /* Motor Inverts */
    public boolean DRIVE_MOTOR_INVERT = true;
    public boolean ANGLE_MOTOR_INVERT = true;

    /* Angle Encoder Invert */
    public boolean CANCODER_INVERT = false;

    // public  double FALCON_TRAP_FREE_SPEED = 6380.0;
    public double KRAKEN_TRAP_FREE_SPEED = 6000.0;
    public double MAX_ACCELERATION_METERS_PER_SECOND = 3.0; // 4.3;
    public double LINEAR_METERS_PER_WHEEL_ROTATIONS = WHEEL_DIAMETER_METERS * Math.PI;

    /** Measured Max Speed: 4.724 MPS */
    public double MAX_VELOCITY_METERS_PER_SECOND =
        (KRAKEN_TRAP_FREE_SPEED / 60.0 * DRIVE_GEAR_RATIO * LINEAR_METERS_PER_WHEEL_ROTATIONS);

    /** Measured Max Angular Speed: 12.65 RadPS */
    public double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.3;

    public SwerveModuleConfig FRONT_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET);
    public SwerveModuleConfig FRONT_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);
    public SwerveModuleConfig BACK_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);
    public SwerveModuleConfig BACK_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

    public SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    public TrajectoryConfig TRAJECTORY_CONFIG =
        new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND)
            .setKinematics(SWERVE_KINEMATICS);
    public TrajectoryConfig REVERSE_TRAJECTORY_CONFIG =
        new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND)
            .setKinematics(SWERVE_KINEMATICS)
            .setReversed(true);
    public TrajectoryConfig TESTING_TRAJECTORY_CONFIG =
        new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND * 0.65, MAX_ACCELERATION_METERS_PER_SECOND * 0.65)
            .setKinematics(SWERVE_KINEMATICS);
    public TrajectoryConfig TESTING_REVERSE_TRAJECTORY_CONFIG =
        new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND * 0.65, MAX_ACCELERATION_METERS_PER_SECOND * 0.65)
            .setKinematics(SWERVE_KINEMATICS)
            .setReversed(true);
  }

  public static class Colors {
    public static final Color RED = new Color(255, 0, 0);
    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color YELLOW = new Color(255, 255, 0);
    public static final Color BLUE = new Color(0, 0, 255);
    public static final Color OFF = new Color(0, 0, 0);
    public static final Color ORANGE = new Color(255, 100, 25);
    public static final Color PINK = new Color(255, 105, 180);
    public static final Color PURPLE = new Color(128, 0, 128);
    public static final Color CYAN = new Color(0, 255, 255);
    public static final Color HOT_PINK = new Color(255, 105, 180);
  }
}
