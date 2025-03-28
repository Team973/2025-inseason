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
    public static final int ENCODER_CAN_ID = 32;

    public static final double ARM_ROTATIONS_PER_MOTOR_ROTATIONS = (10.0 / 84.0) * (16.0 / 108.0);
    public static final double ENCODER_OFFSET_ROTATIONS = -0.0027;

    public static final double ARM_LENGTH_METERS = 0.451104;
    public static final double ARM_MAX_ANGLE_DEG = 75.0;
    public static final double ARM_MIN_ANGLE_DEG = -80.0;
  }

  public static class ElevatorInfo {
    public static final int HALL_SENSOR_ID = 0;
    public static final double MAX_HEIGHT_METERS = 0.7112;
    public static final double FLOOR_TO_ELEVATOR_ZERO_METERS = 0.8636;
  }

  public static class ClawInfo {
    public static final int RIGHT_MOTOR_ID = 36;
    public static final int LEFT_MOTOR_ID = 35;
    public static final int CONVEYOR_MOTOR_ID = 34;

    public static final int CONVEYOR_BACK_SENSOR_ID = 2;
    public static final int CONVEYOR_FRONT_SENSOR_ID = 3;
    public static final int CLAW_ALGAE_CAN_ID = 48;
  }

  public static class WristInfo {
    public static final int MOTOR_CAN_ID = 31;
    public static final int ENCODER_CAN_ID = 35;

    public static final double ENCODER_OFFSET_ROTATIONS = 0.2524;
    public static final double WRIST_ROTATIONS_PER_MOTOR_ROTATIONS =
        10.0 / 46.0 * 14.0 / 72.0 * 34.0 / 60.0;

    public static final double MAX_ANGLE_DEG = -13.0;
    public static final double MIN_ANGLE_DEG = -195.0;
  }

  public static final class ClimbInfo {
    public static final double MOTOR_ROT_PER_CLIMB_ROT =
        1.0 / ((10.0 / 64.0) * (20.0 / 72.0) * (20.0 / 84.0) * (9.0 / 30.0));
  }

  public static class SignalerInfo {
    public static final int CRASH_SIGNALER_PRIORITY = 5;
    public static final int CLIMB_STOP_PRIORITY = 6;
    public static final int CLIMB_HORIZONTAL_PRIORITY = 7;
    public static final int ARM_TARGET_OUT_OF_BOUNDS_PRIORITY = 8;
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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = fromBotVersion(-743.027, 264.902);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = fromBotVersion(-684.931, 153.721);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 7;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = fromBotVersion(-1249.628, 50.713);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = fromBotVersion(-490.517, 100.107);

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
    public static final double ANGLE_KP = 5.5; // 6.5;
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
