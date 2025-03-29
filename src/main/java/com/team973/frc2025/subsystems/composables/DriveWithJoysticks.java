package com.team973.frc2025.subsystems.composables;

import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.DriveController.RotationControl;
import com.team973.lib.util.DriveComposable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveWithJoysticks extends DriveComposable {
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

  private RotationControl m_rotationControl = RotationControl.OpenLoop;
  private final PIDController m_rotationController = new PIDController(0.1, 0.0, 0.002);

  private double m_lastRot = 0.0;

  private Rotation2d m_targetRobotAngle = new Rotation2d();

  private boolean m_holdingAngle = false;

  private double m_xAxis = 0.0;
  private double m_yAxis = 0.0;
  private double m_rot = 0.0;

  private Rotation2d m_currentYaw = new Rotation2d();
  private Rotation2d m_angularVelocity = new Rotation2d();

  public DriveWithJoysticks() {}

  public void setRotationControl(RotationControl rotationControl) {
    m_rotationControl = rotationControl;
    if (m_rotationControl == RotationControl.OpenLoop) {
      m_holdingAngle = false;
    }
  }

  public void setHeldAngle(Rotation2d angle) {
    m_rotationControl = RotationControl.ClosedLoop;
    m_holdingAngle = true;
    m_targetRobotAngle = angle;
  }

  public void reset(Rotation2d currentYaw) {
    m_targetRobotAngle = currentYaw;
  }

  public void updateInput(double xAxis, double yAxis, double rotAxis) {
    m_xAxis = xAxis;
    m_yAxis = yAxis;
    m_rot = rotAxis;
  }

  public void updateAngle(Rotation2d currentYaw, Rotation2d angularVelocity) {
    m_currentYaw = currentYaw;
    m_angularVelocity = angularVelocity;
  }

  public void init(ChassisSpeeds previousChassisSpeeds) {}

  public void exit() {}

  @Override
  public ChassisSpeeds getOutput() {
    final double xSpeed =
        -MathUtil.applyDeadband(m_xAxis, 0.1) * DriveInfo.MAX_VELOCITY_METERS_PER_SECOND;
    final double ySpeed =
        -MathUtil.applyDeadband(m_yAxis, 0.1) * DriveInfo.MAX_VELOCITY_METERS_PER_SECOND;

    double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_rot, 0.09))
            * DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            * 0.7;

    if (m_lastRot != 0.0 && rot == 0.0 && !m_holdingAngle) {
      // m_targetRobotAngle = currentYaw;
      // Correct for latency in robot rotation and measurement.
      m_targetRobotAngle = m_currentYaw.plus(m_angularVelocity.times(0.03));
      setRotationControl(RotationControl.ClosedLoop);
    } else if (rot != 0.0 && !m_holdingAngle) {
      setRotationControl(RotationControl.OpenLoop);
    }

    m_lastRot = rot;

    if (m_rotationControl == RotationControl.ClosedLoop) {
      double diff = m_targetRobotAngle.minus(m_currentYaw).getDegrees();
      if (diff > 180) {
        diff -= 360;
      } else if (diff < -180) {
        diff += 360;
      }

      rot =
          m_rotationController.calculate(
              m_currentYaw.getDegrees(), m_currentYaw.getDegrees() + diff);
    }

    return new ChassisSpeeds(xSpeed, ySpeed, rot);
  }
}
