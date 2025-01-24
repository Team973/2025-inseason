package com.team973.frc2025.subsystems.composables;

import com.team973.frc2025.shared.RobotInfo.DriveInfo;
import com.team973.lib.util.DriveComposable;
import com.team973.lib.util.GreyHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveWithLimelight extends DriveComposable {
  private final GreyHolonomicDriveController m_controller;
  private final Pose2d m_targetPoseRelativeToAprilTag;

  private Pose2d m_currentPose;

  public DriveWithLimelight(Pose2d targetPoseRelativeToAprilTag) {
    m_controller =
        new GreyHolonomicDriveController(
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new ProfiledPIDController(
                2,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    DriveInfo.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3.0)));
    m_targetPoseRelativeToAprilTag = targetPoseRelativeToAprilTag;

    m_currentPose = new Pose2d();
  }

  public void updatePose(Pose2d pose) {
    m_currentPose = pose;
  }

  public void setTargetIDs(int... ids) {}

  private double getTargetRot() {
    return 0;
  }

  public ChassisSpeeds getOutput() {
    return new ChassisSpeeds(
        0,
        0,
        m_controller
            .getThetaController()
            .calculate(m_currentPose.getRotation().getDegrees(), getTargetRot()));
  }
}
