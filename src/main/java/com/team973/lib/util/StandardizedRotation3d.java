package com.team973.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class StandardizedRotation3d extends Rotation3d {

  /** Constructs a Rotation3d with a default angle of 0 degrees. */
  public StandardizedRotation3d() {
    super();
  }

  /**
   * Constructs a Rotation3d2 from a Rotation3d.
   *
   * @param rotation3d The Rotation3d to copy.
   */
  public StandardizedRotation3d(Rotation3d rotation3d) {
    this(rotation3d.getQuaternion());
  }

  /**
   * Constructs a Rotation3d from three Rotation2d.
   *
   * @param x The Rotation2d around the x axis (roll).
   * @param y The Rotation2d around the y axis (pitch).
   * @param z The Rotation2d around the z axis (yaw).
   */
  public StandardizedRotation3d(Rotation2d x, Rotation2d y, Rotation2d z) {
    this(x.getRadians(), y.getRadians(), z.getRadians());
  }

  /**
   * Constructs a Rotation3d from a quaternion.
   *
   * @param q The quaternion.
   */
  public StandardizedRotation3d(Quaternion q) {
    super(q);
  }

  /**
   * Constructs a Rotation3d from extrinsic roll, pitch, and yaw.
   *
   * <p>Extrinsic rotations occur in that order around the axes in the fixed global frame rather
   * than the body frame.
   *
   * <p>Angles are measured counterclockwise with the rotation axis pointing "out of the page". If
   * you point your right thumb along the positive axis direction, your fingers curl in the
   * direction of positive rotation.
   *
   * @param roll The counterclockwise rotation angle around the X axis (roll) in radians.
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch) in radians.
   * @param yaw The counterclockwise rotation angle around the Z axis (yaw) in radians.
   */
  public StandardizedRotation3d(double roll, double pitch, double yaw) {
    super(roll, pitch, yaw);
  }

  /**
   * Constructs a Rotation3d with the given rotation vector representation. This representation is
   * equivalent to axis-angle, where the normalized axis is multiplied by the rotation around the
   * axis in radians.
   *
   * @param rvec The rotation vector.
   */
  public StandardizedRotation3d(Vector<N3> rvec) {
    super(rvec);
  }

  /**
   * Constructs a Rotation3d with the given axis-angle representation. The axis doesn't have to be
   * normalized.
   *
   * @param axis The rotation axis.
   * @param angleRadians The rotation around the axis in radians.
   */
  public StandardizedRotation3d(Vector<N3> axis, double angleRadians) {
    super(axis, angleRadians);
  }

  /**
   * Constructs a Rotation3d from a rotation matrix.
   *
   * @param rotationMatrix The rotation matrix.
   * @throws IllegalArgumentException if the rotation matrix isn't special orthogonal.
   */
  public StandardizedRotation3d(Matrix<N3, N3> rotationMatrix) {
    super(rotationMatrix);
  }

  /**
   * Constructs a Rotation3d that rotates the initial vector onto the final vector.
   *
   * <p>This is useful for turning a 3D vector (final) into an orientation relative to a coordinate
   * system vector (initial).
   *
   * @param initial The initial vector.
   * @param last The final vector.
   */
  public StandardizedRotation3d(Vector<N3> initial, Vector<N3> last) {
    super(initial, last);
  }

  /**
   * Get the Rotation2d representing the yaw of the rotation.
   *
   * @return The Rotation2d representing the yaw of the rotation.
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromRadians(getZ());
  }

  /**
   * Get the Rotation2d representing the pitch of the rotation.
   *
   * @return The Rotation2d representing the pitch of the rotation.
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(getX());
  }

  /**
   * Get the Rotation2d representing the roll of the rotation.
   *
   * @return The Rotation2d representing the roll of the rotation.
   */
  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(getY());
  }

  @Override
  public StandardizedRotation3d plus(Rotation3d other) {
    return new StandardizedRotation3d(super.plus(other));
  }

  @Override
  public StandardizedRotation3d minus(Rotation3d other) {
    return new StandardizedRotation3d(super.minus(other));
  }

  @Override
  public StandardizedRotation3d unaryMinus() {
    return new StandardizedRotation3d(super.unaryMinus());
  }

  @Override
  public StandardizedRotation3d times(double scalar) {
    return new StandardizedRotation3d(super.times(scalar));
  }

  @Override
  public StandardizedRotation3d div(double scalar) {
    return new StandardizedRotation3d(super.div(scalar));
  }

  @Override
  public StandardizedRotation3d rotateBy(Rotation3d other) {
    return new StandardizedRotation3d(other.getQuaternion().times(getQuaternion()));
  }

  @Override
  public String toString() {
    return String.format("StandardizedRotation3d(%s)", getQuaternion());
  }

  @Override
  public StandardizedRotation3d interpolate(Rotation3d endValue, double t) {
    return new StandardizedRotation3d(super.interpolate(endValue, t));
  }
}
