package com.team973.frc2025.subsystems.composables;

import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.frc2025.subsystems.DriveController.RotationControl;
import com.team973.lib.util.DriveComposable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithJoysticks implements DriveComposable {
  private ChassisSpeeds m_driveInput;
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

  private RotationControl m_rotationControl = RotationControl.OpenLoop;
  private final PIDController m_rotationController = new PIDController(0.1, 0.0, 0.002);

  private double m_lastRot = 0.0;

  private Rotation2d m_targetRobotAngle = new Rotation2d();

  private boolean m_holdingAngle = false;

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

  public void updateJoystickInput(
      double xAxis,
      double yAxis,
      double rotAxis,
      Rotation2d currentYaw,
      Rotation2d angularVelocity) {
    final double xSpeed =
        -MathUtil.applyDeadband(xAxis, 0.1) * DriveInfo.MAX_VELOCITY_METERS_PER_SECOND;
    final double ySpeed =
        -MathUtil.applyDeadband(yAxis, 0.1) * DriveInfo.MAX_VELOCITY_METERS_PER_SECOND;

    double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(rotAxis, 0.09))
            * DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            * 0.7;

    if (m_lastRot != 0.0 && rot == 0.0 && !m_holdingAngle) {
      // m_targetRobotAngle = currentYaw;
      // Correct for latency in robot rotation and measurement.
      m_targetRobotAngle = currentYaw.plus(angularVelocity.times(0.03));
      setRotationControl(RotationControl.ClosedLoop);
    } else if (rot != 0.0 && !m_holdingAngle) {
      setRotationControl(RotationControl.OpenLoop);
    }

    m_lastRot = rot;

    if (m_rotationControl == RotationControl.ClosedLoop) {
      double diff = m_targetRobotAngle.minus(currentYaw).getDegrees();
      if (diff > 180) {
        diff -= 360;
      } else if (diff < -180) {
        diff += 360;
      }

      SmartDashboard.putNumber("Angle Diff", diff);

      rot = m_rotationController.calculate(currentYaw.getDegrees(), currentYaw.getDegrees() + diff);
    }

    SmartDashboard.putNumber("Robot Rotation", rot);
    SmartDashboard.putNumber("Target Robot Angle", m_targetRobotAngle.getDegrees());
    SmartDashboard.putNumber(
        "drive/driveWithJoysticks/Current Robot Angle", currentYaw.getDegrees());
    SmartDashboard.putString("Rotation Control", String.valueOf(m_rotationControl));

    m_driveInput = new ChassisSpeeds(xSpeed, ySpeed, rot);
  }

  @Override
  public ChassisSpeeds getOutput() {
    return m_driveInput;
  }
}
