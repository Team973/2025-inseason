package com.team973.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.DoubleSupplier;

public class JoystickField {
  private final DoubleSupplier m_xAxisSupplier;
  private final DoubleSupplier m_yAxisSupplier;

  public class Range {
    private final Rotation2d m_centerAngle;
    private final Rotation2d m_maxAngleOffsetFromCenter;
    private final double m_minDistFromCenter;

    public Range(
        Rotation2d centerAngle, Rotation2d maxAngleOffsetFromCenter, double minDistFromCenter) {
      m_centerAngle = centerAngle;
      m_maxAngleOffsetFromCenter = maxAngleOffsetFromCenter;
      m_minDistFromCenter = minDistFromCenter;
    }

    public boolean isActive() {
      return ((getAngleDeg()
                      >= wrapAngle(m_centerAngle.minus(m_maxAngleOffsetFromCenter).getDegrees())
                  && getAngleDeg() < wrapAngle(m_centerAngle.getDegrees()))
              || (getAngleDeg()
                      < wrapAngle(m_centerAngle.plus(m_maxAngleOffsetFromCenter).getDegrees())
                  && getAngleDeg() >= wrapAngle(m_centerAngle.getDegrees())))
          && getDistFromCenter() >= m_minDistFromCenter;
    }
  }

  public JoystickField(DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier) {
    m_xAxisSupplier = xAxisSupplier;
    m_yAxisSupplier = yAxisSupplier;
  }

  private double wrapAngle(double angle) {
    if (angle > 360) {
      return angle - 360;
    } else if (angle < 0) {
      return angle + 360;
    }

    return angle;
  }

  public Range range(
      Rotation2d centerAngle, Rotation2d maxAngleOffsetFromCenter, double minDistFromCenter) {
    return new Range(centerAngle, maxAngleOffsetFromCenter, minDistFromCenter);
  }

  public Translation2d getTranslation() {
    return new Translation2d(m_xAxisSupplier.getAsDouble(), m_yAxisSupplier.getAsDouble());
  }

  public double getAngleDeg() {
    return wrapAngle(getTranslation().getAngle().minus(Rotation2d.kCW_90deg).getDegrees());
  }

  public double getDistFromCenter() {
    return getTranslation().getDistance(Translation2d.kZero);
  }
}
