package com.team973.frc2025.shared;

import com.team973.lib.util.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/** Robot info, specs, dimensions. */
public final class RobotInfo {
  public static final String CANIVORE_CANBUS = "Canivore"; // "Canivore";
  public static final String ROBORIO_CANBUS = "";

  public static class DriveInfo {
    public static final int STATUS_SIGNAL_FREQUENCY = 200;

    public static final int PIGEON_ID = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 78.041; // 147.0;
    // 179.207 + 100.362;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -128.522; // -36.278 + 105.915;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 7;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 65.993; // 94.283 + 31.135;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -55.325; // -33.722 + 23.526;

    public static final double DRIVE_GEAR_RATIO = ((14.0 / 42.0) * (32.0 / 14.0) * (15.0 / 45.0));
    public static final double ANGLE_GEAR_RATIO = ((10.0 / 26.0) * (14.0 / 72.0));

    public static final double WHEEL_DIAMETER_METERS = 0.1016 * (5.82 / 6.0) * (1.55 / 2.0);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    /**
     * The left-to-right distance between the drivetrain wheels Should be measured from center to
     * center.
     */
    public static final double TRACKWIDTH_METERS = 0.61595;

    /**
     * The front-to-back distance between the drivetrain wheels. Should be measured from center to
     * center.
     */
    public static final double WHEELBASE_METERS = 0.57785;

    public static final double OPENLOOP_RAMP = 0.0;
    public static final double CLOSEDLOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 8.0; // 10.0 at SD // 6.0
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_KF = 0.0;
    public static final double ANGLE_KV = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.25; // 0.35;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.148; // 0.18;

    /* Motor Inverts */
    public static final boolean DRIVE_MOTOR_INVERT = true;
    public static final boolean ANGLE_MOTOR_INVERT = true;

    /* Angle Encoder Invert */
    public static final boolean CANCODER_INVERT = false;

    // public static final double FALCON_TRAP_FREE_SPEED = 6380.0;
    public static final double KRAKEN_TRAP_FREE_SPEED = 6000.0;
    public static final double MAX_ACCELORATION_METERS_PER_SECOND = 3.0; // 4.3;
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        (KRAKEN_TRAP_FREE_SPEED / 60.0 * DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI);

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.3; // 11.5;

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
        new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELORATION_METERS_PER_SECOND)
            .setKinematics(SWERVE_KINEMATICS);
    public static final TrajectoryConfig REVERSE_TRAJECTORY_CONFIG =
        new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELORATION_METERS_PER_SECOND)
            .setKinematics(SWERVE_KINEMATICS)
            .setReversed(true);
    public static final TrajectoryConfig TESTING_TRAJECTORY_CONFIG =
        new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND * 0.65, MAX_ACCELORATION_METERS_PER_SECOND * 0.65)
            .setKinematics(SWERVE_KINEMATICS);
    public static final TrajectoryConfig TESTING_REVERSE_TRAJECTORY_CONFIG =
        new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND * 0.65, MAX_ACCELORATION_METERS_PER_SECOND * 0.65)
            .setKinematics(SWERVE_KINEMATICS)
            .setReversed(true);
  }
}
