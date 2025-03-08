package com.team973.frc2025.shared;

import com.team973.lib.util.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Color;

/** Robot info, specs, dimensions. */
public final class RobotInfo {

  public static enum BotVersion {
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
  public static final String ROBORIO_CANBUS = "";

  public static class ArmInfo {
    public static final int HALL_SENSOR_ID = 1;

    public static final double ARM_HOMING_POSTION_DEG = -90.0;

    public static final double LEVEL_FOUR_POSITION_DEG = 64.0; // 79
    public static final double LEVEL_THREE_POSITION_DEG = 64.0;
    public static final double LEVEL_TWO_POSITION_DEG = -58.0; // -70.0;
    public static final double LEVEL_ONE_POSITION_DEG = -59.0; // -70.0;
    public static final double CORAL_STOW_POSITION_DEG = -92.0;

    public static final double ALGAE_HIGH_POSITION_DEG = 38.0;
    public static final double ALGAE_LOW_POSITION_DEG = -61.0;
    public static final double ALGAE_STOW_POSITION_DEG = -80.0;

    public static final double ARM_ROTATIONS_PER_MOTOR_ROTATIONS = (10.0 / 74.0) * (18.0 / 84.0);
    public static final double CENTER_GRAVITY_OFFSET_DEG = 3;
    public static final double FEED_FORWARD_MAX_VOLT = 0.6; // 0.5;

    public static final double ARM_KS = 0.0;
    public static final double ARM_KV = 0.0;
    public static final double ARM_KA = 0.0;
    public static final double ARM_KP = 50.0;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.0;

    public static final double ARM_MOTION_MAGIC_CRUISE_VELOCITY = 110.0; // 64.0;
    public static final double ARM_MOTION_MAGIC_ACCELERATION = 80.0; // 80.0;
    public static final double ARM_MOTION_MAGIC_JERK = 1000.0;

    public static final double ARM_SATOR_CURRENT_LIMIT = 60.0;
    public static final boolean ARM_SATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double ARM_SUPPLY_CURRENT_LIMIT = 30.0;
    public static final boolean ARM_SUPPLY_CURRENT_LIMIT_ENABLE = true;

    public static final double ARM_PEAK_FORDWARD_VOLTAGE = 12.0;
    public static final double ARM_PEAK_REVERSE_VOLTAGE = -12.0;
  }

  public static class ElevatorInfo {
    public static final int HALL_SENSOR_ID = 0;
    public static final double MOTOR_GEAR_RATIO = 10.0 / 56.0;

    public static final double LEVEL_1 = 2.0;
    public static final double LEVEL_2 = 12.0;
    public static final double LEVEL_3 = 2.5;
    public static final double LEVEL_4 = 27.5;
    public static final double CORAL_STOW = 0.0;

    public static final double ALGAE_HIGH = 21.0;
    public static final double ALGAE_LOW = 25.5;
    public static final double ALGAE_STOW = 5.0;

    public static final double ELEVATOR_MM_KS = 0.0;
    public static final double ELEVATOR_MM_KV = 0.15;
    public static final double ELEVATOR_MM_KA = 0.01;
    public static final double ELEVATOR_MM_KP = 4.0;
    public static final double ELEVATOR_MM_KI = 0.0;
    public static final double ELEVATOR_MM_KD = 0.0;

    public static final double ELEVATOR_V_KS = 0.0;
    public static final double ELEVATOR_V_KV = 0.0;
    public static final double ELEVATOR_V_KA = 0.0;
    public static final double ELEVATOR_V_KP = 125.0;
    public static final double ELEVATOR_V_KI = 0.0;
    public static final double ELEVATOR_V_KD = 0.0;

    public static final double ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY =
        32.0; // 32.0; // 64; // 32; // 16;
    public static final double ELEVATOR_MOTION_MAGIC_ACCELERATION =
        300.0; // 40.0; // 500; // 40; // 20;
    public static final double ELEVATOR_MOTION_MAGIC_JERK = 2000.0;

    public static final double ELEVATOR_SATOR_CURRENT_LIMIT = 60.0;
    public static final boolean ELEVATOR_SATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 40.0;
    public static final boolean ELEVATOR_SUPPLY_CURRENT_LIMIT_ENABLE = true;

    public static final double ELEVATOR_PEAK_FORDWARD_VOLTAGE = 12.0;
    public static final double ELEVATOR_PEAK_REVERSE_VOLTAGE = -12.0;
    public static final double ELEVATOR_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.02;
  }

  public static class ClawInfo {
    public static final int RIGHT_MOTOR_ID = 36;
    public static final int LEFT_MOTOR_ID = 35;
    public static final int CONVEYOR_MOTOR_ID = 34;

    public static final int CONVEYOR_BACK_SENSOR_ID = 2;
    public static final int CONVEYOR_FRONT_SENSOR_ID = 3;
    public static final int CLAW_ALGAE_CAN_ID = 48;

    public static final double CLAW_KS = 0.0;
    public static final double CLAW_KV = 0.125 * 10.0 / 10.5;
    public static final double CLAW_KA = 0.0;
    public static final double CLAW_KP = 0.3;
    public static final double CLAW_KI = 0.0;
    public static final double CLAW_KD = 0.0;

    public static final double CLAW_SATOR_CURRENT_LIMIT = 30.0;
    public static final boolean CLAW_SATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double CLAW_SUPPLY_CURRENT_LIMIT = 20.0;
    public static final boolean CLAW_SUPPLY_CURRENT_LIMIT_ENABLE = true;

    public static final double CLAW_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.02;
  }

  public static class ClimbInfo {
    public static final double CLIMB_KS = 0.0;
    public static final double CLIMB_KV = 0.0;
    public static final double CLIMB_KA = 0.0;
    public static final double CLIMB_KP = 6.4;
    public static final double CLIMB_KI = 0.0;
    public static final double CLIMB_KD = 0.04;

    public static final double CLIMB_MOTION_MAGIC_CRUISE_VELOCITY = 32.0;
    public static final double CLIMB_MOTION_MAGIC_ACCELERATION = 32.0;
    public static final double CLIMB_MOTION_MAGIC_JERK = 160.0;

    public static final double CLIMB_SATOR_CURRENT_LIMIT = 30.0;
    public static final boolean CLIMB_SATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double CLIMB_SUPPLY_CURRENT_LIMIT = 20.0;
    public static final boolean CLIMB_SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double CLIMB_VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.02;

    public static final double CLIMB_PEAK_FORDWARD_VOLTAGE = 2.0;
    public static final double CLIMB_PEAK_REVERSE_VOLTAGE = -2.0;

    public static final double MOTOR_ROT_PER_CLIMB_ROT =
        1.0 / ((10.0 / 64.0) * (20.0 / 72.0) * (20.0 / 84.0) * (9.0 / 30.0));
  }

  public static class SignalerInfo {
    public static final int CRASH_SIGNALER_PRIORITY = 5;
    public static final int CLIMB_STOP_PRIORITY = 6;
    public static final int CLIMB_HORIZONTAL_PRIORITY = 7;
    public static final int ELEVATOR_HALL_SENSOR_SIGNALER_PRIORITY = 10;
    public static final int ARM_HALL_SENSOR_SIGNALER_PRIORTY = 11;
    public static final int PEICE_IN_CLAW_SIGNALER_PRIORTY = 50;
    public static final int LOW_BATTER_SIGNALER_PRIORTY = 99;
    public static final int OFF_SIGNALER_PRIORTY = 100;
  }

  public static class DriveInfo {
    public static final int STATUS_SIGNAL_FREQUENCY = 200;

    public static final int PIGEON_ID = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = fromBotVersion(-311.133, 264.902);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = fromBotVersion(-328.096, 153.721);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 7;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = fromBotVersion(-171.299, 50.713);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = fromBotVersion(156.533, 100.107);

    public static final double DRIVE_GEAR_RATIO =
        (10.0 / 54.0) * (40.0 / 16.0) * (15.0 / 45.0); // x3:10, 6.48:1

    public static final double ANGLE_GEAR_RATIO = (10.0 / 22.0) * (16.0 / 88.0); // 12.1:1

    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    /**
     * The left-to-right distance between the drivetrain wheels Should be measured from center to
     * center.
     */
    public static final double TRACKWIDTH_METERS = fromBotVersion(0.635, 0.5334);

    /**
     * The front-to-back distance between the drivetrain wheels. Should be measured from center to
     * center.
     */
    public static final double WHEELBASE_METERS = fromBotVersion(0.635, 0.5334);

    public static final double OPEN_LOOP_RAMP = 0.0;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 6.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_KF = 0.0;
    public static final double ANGLE_KV = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.38;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.12;

    /* Motor Inverts */
    public static final boolean DRIVE_MOTOR_INVERT = true;
    public static final boolean ANGLE_MOTOR_INVERT = true;

    /* Angle Encoder Invert */
    public static final boolean CANCODER_INVERT = false;

    // public static final double FALCON_TRAP_FREE_SPEED = 6380.0;
    public static final double KRAKEN_TRAP_FREE_SPEED = 6000.0;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND = 3.0; // 4.3;
    public static final double LINEAR_METERS_PER_WHEEL_ROTATIONS = WHEEL_DIAMETER_METERS * Math.PI;

    /** Measured Max Speed: 4.724 MPS */
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        (KRAKEN_TRAP_FREE_SPEED / 60.0 * DRIVE_GEAR_RATIO * LINEAR_METERS_PER_WHEEL_ROTATIONS);

    /** Measured Max Angular Speed: 12.65 RadPS */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.3;

    public static final SwerveModuleConfig FRONT_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET);
    public static final SwerveModuleConfig FRONT_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);
    public static final SwerveModuleConfig BACK_LEFT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);
    public static final SwerveModuleConfig BACK_RIGHT_CONSTANTS =
        new SwerveModuleConfig(
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    public static final TrajectoryConfig TRAJECTORY_CONFIG =
        new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND)
            .setKinematics(SWERVE_KINEMATICS);
    public static final TrajectoryConfig REVERSE_TRAJECTORY_CONFIG =
        new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND)
            .setKinematics(SWERVE_KINEMATICS)
            .setReversed(true);
    public static final TrajectoryConfig TESTING_TRAJECTORY_CONFIG =
        new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND * 0.65, MAX_ACCELERATION_METERS_PER_SECOND * 0.65)
            .setKinematics(SWERVE_KINEMATICS);
    public static final TrajectoryConfig TESTING_REVERSE_TRAJECTORY_CONFIG =
        new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND * 0.65, MAX_ACCELERATION_METERS_PER_SECOND * 0.65)
            .setKinematics(SWERVE_KINEMATICS)
            .setReversed(true);
  }

  public static class Colors {
    public static final Color RED = new Color(255, 0, 0);
    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color BLUE = new Color(0, 0, 255);
    public static final Color OFF = new Color(0, 0, 0);
    public static final Color ORANGE = new Color(255, 100, 25);
    public static final Color PINK = new Color(255, 105, 180);
    public static final Color PURPLE = new Color(128, 0, 128);
    public static final Color CYAN = new Color(0, 255, 255);
    public static final Color HOT_PINK = new Color(255, 105, 180);
  }
}
