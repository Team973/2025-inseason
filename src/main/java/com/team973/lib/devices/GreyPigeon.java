package com.team973.lib.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team973.frc2025.shared.RobotInfo;
import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.lib.util.Logger;
import com.team973.lib.util.StandardizedRotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.ArrayList;

/** GreyPigeon - Pigeon2 wrapper class */
public class GreyPigeon {
  public final Pigeon2 m_pigeon;
  private StandardizedRotation3d m_offset;

  private final Logger m_logger;

  private static final Rotation2d DEFAULT_LEVEL_TOLERANCE = Rotation2d.fromDegrees(2.0);

  private double m_yawOffsetDegrees = 0.0;

  private final StatusSignal<Angle> m_yawStatusSignal;
  private final StatusSignal<Angle> m_pitchStatusSignal;
  private final StatusSignal<Angle> m_rollStatusSignal;
  private final StatusSignal<AngularVelocity> m_yawVelocityStatusSignal;
  private final StatusSignal<AngularVelocity> m_pitchVelocityStatusSignal;
  private final StatusSignal<AngularVelocity> m_rollVelocityStatusSignal;

  private final ArrayList<StatusSignal<Angle>> m_angleStatusSignals =
      new ArrayList<StatusSignal<Angle>>(2);

  private final ArrayList<StatusSignal<AngularVelocity>> m_angularVelocityStatusSignals =
      new ArrayList<StatusSignal<AngularVelocity>>(2);

  /** Creates a new GreyPigeon. */
  public GreyPigeon(Logger logger) {
    m_pigeon = new Pigeon2(DriveInfo.PIGEON_ID, RobotInfo.CANIVORE_CANBUS);
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());

    m_logger = logger;

    m_yawStatusSignal = m_pigeon.getYaw();
    m_pitchStatusSignal = m_pigeon.getPitch();
    m_rollStatusSignal = m_pigeon.getRoll();
    m_yawVelocityStatusSignal = m_pigeon.getAngularVelocityZWorld();
    m_pitchVelocityStatusSignal = m_pigeon.getAngularVelocityYWorld();
    m_rollVelocityStatusSignal = m_pigeon.getAngularVelocityXWorld();

    m_angleStatusSignals.add(m_yawStatusSignal);
    m_angularVelocityStatusSignals.add(m_pitchVelocityStatusSignal);

    reset();
  }

  public StandardizedRotation3d getOffset() {
    return m_offset;
  }

  public void setOffset(StandardizedRotation3d offset) {
    m_offset = offset;
  }

  public ArrayList<StatusSignal<Angle>> getAngleStatusSignals() {
    return m_angleStatusSignals;
  }

  public ArrayList<StatusSignal<AngularVelocity>> getAngularVelocityStatusSignals() {
    return m_angularVelocityStatusSignals;
  }

  /**
   * Returns a StandardizedRotation3d object containing the yaw, pitch, and roll from the Pigeon2.
   *
   * @return A StandardizedRotation3d object containing the adjusted yaw, pitch, and roll from the
   *     Pigeon2.
   */
  public StandardizedRotation3d getRotation() {
    return getRawRotation().minus(m_offset);
  }

  /**
   * Returns a StandardizedRotation3d object containing the raw yaw, pitch, and roll from the
   * Pigeon2.
   *
   * @return A StandardizedRotation3d object containing the raw yaw, pitch, and roll from the
   *     Pigeon2.
   */
  public StandardizedRotation3d getRawRotation() {
    return new StandardizedRotation3d(getRawRoll(), getRawPitch(), getRawYaw());
  }

  /**
   * Returns the yaw from the Pigeon2 with the offset applied.
   *
   * @return The yaw from the Pigeon2 with the offset applied.
   */
  public synchronized Rotation2d getYaw() {
    return getRawYaw().minus(m_offset.getYaw());
    // return Rotation2d.fromDegrees(m_pigeon.getYaw().getValueAsDouble() - m_yawOffsetDegrees);
  }

  /**
   * Returns the pitch from the Pigeon2 with the offset applied.
   *
   * @return The pitch from the Pigeon2 with the offset applied.
   */
  public Rotation2d getPitch() {
    return getRawPitch().minus(m_offset.getPitch());
  }

  /**
   * Returns the roll from the Pigeon2 with the offset applied.
   *
   * @return The roll from the Pigeon2 with the offset applied.
   */
  public Rotation2d getRoll() {
    return getRawRoll().minus(m_offset.getRoll());
  }

  /**
   * Returns the raw yaw from the Pigeon2.
   *
   * @return The raw yaw from the Pigeon2.
   */
  public Rotation2d getRawYaw() {
    double compensatedAngle =
        BaseStatusSignal.getLatencyCompensatedValue(m_yawStatusSignal, m_yawVelocityStatusSignal)
            .magnitude();
    return Rotation2d.fromDegrees(compensatedAngle);
    // return Rotation2d.fromDegrees(m_pigeon.getYaw().getValueAsDouble());
  }

  /**
   * Returns the raw pitch from the Pigeon2.
   *
   * @return The raw pitch from the Pigeon2.
   */
  public Rotation2d getRawPitch() {
    double compensatedAngle =
        BaseStatusSignal.getLatencyCompensatedValue(
                m_pitchStatusSignal, m_pitchVelocityStatusSignal)
            .magnitude();
    return Rotation2d.fromDegrees(compensatedAngle);
  }

  /**
   * Returns the raw roll from the Pigeon2.
   *
   * @return The raw roll from the Pigeon2.
   */
  public Rotation2d getRawRoll() {
    double compensatedAngle =
        BaseStatusSignal.getLatencyCompensatedValue(m_rollStatusSignal, m_rollVelocityStatusSignal)
            .magnitude();
    return Rotation2d.fromDegrees(compensatedAngle);
  }

  /**
   * Returns the normalized yaw from the Pigeon2 with the offset applied.
   *
   * @return The normalized yaw from the Pigeon2 with the offset applied.
   */
  public Rotation2d getNormalizedYaw() {
    double rawYaw = getYaw().getDegrees();
    double normalizedYaw = Math.IEEEremainder(rawYaw, 360.0);
    if (normalizedYaw < 0) {
      normalizedYaw += 360.0;
    }
    return Rotation2d.fromDegrees(normalizedYaw);
  }

  /**
   * Returns the inclination of the Pigeon2.
   *
   * @return The inclination of the Pigeon2.
   */
  public Rotation2d getInclination() {
    return Rotation2d.fromRadians(
        Math.atan(Math.sqrt(Math.pow(getRoll().getTan(), 2) + Math.pow(getPitch().getTan(), 2))));
  }

  /**
   * Returns the angular velocity of the Pigeon2.
   *
   * @return The angular velocity of the Pigeon2.
   */
  public Rotation2d getAngularVelocity() {
    return Rotation2d.fromDegrees(m_yawVelocityStatusSignal.getValueAsDouble());
  }

  public synchronized void setYawOffset(Rotation2d newYaw) {
    var currentOffset = getOffset();
    m_offset =
        new StandardizedRotation3d(currentOffset.getX(), currentOffset.getY(), newYaw.getRadians());
    // m_yawOffsetDegrees = newYaw.getDegrees();
  }

  /**
   * Returns whether the Pigeon2 is level.
   *
   * @return Whether the Pigeon2 is level.
   */
  public boolean isLevel() {
    return isLevel(DEFAULT_LEVEL_TOLERANCE);
  }

  /**
   * Returns whether the Pigeon2 is level within the specified tolerance.
   *
   * @param tolerance The tolerance in degrees.
   * @return Whether the Pigeon2 is level within the specified tolerance.
   */
  public boolean isLevel(Rotation2d tolerance) {
    return Math.abs(getInclination().getDegrees()) < tolerance.getDegrees();
  }

  /** Resets the offset to the current yaw, pitch, and roll. */
  public void reset() {
    m_offset = getRawRotation();
  }

  public void log() {
    m_logger.log("Pitch", () -> getPitch().getDegrees());
    m_logger.log("Roll", () -> getRoll().getDegrees());
    m_logger.log("Yaw", () -> getYaw().getDegrees());
    m_logger.log("Angular Velocity", () -> getAngularVelocity().getDegrees());
  }
}
