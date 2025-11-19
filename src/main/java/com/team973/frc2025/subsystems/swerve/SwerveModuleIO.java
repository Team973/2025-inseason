package com.team973.frc2025.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.List;

public interface SwerveModuleIO {

  public Rotation2d getCanCoderRotation2d();

  public default void resetToAbsolute() {}

  public SwerveModuleState getState();

  public Rotation2d getAngleMotorRotation2d();

  public List<StatusSignal<?>> allStatusSignals();

  public double getDriveMotorMeters();

  public SwerveModulePosition getPosition();

  public double getDriveStatorCurrent();

  public double getDriveSupplyCurrent();

  public double getTurnStatorCurrent();

  public double getTurnSupplyCurrent();

  public default void setDesiredState(SwerveModuleState desiredState) {}

  public default void setDesiredState(SwerveModuleState desiredState, boolean ignoreJitter) {}

  public default void driveBrake() {}

  public default void driveNeutral() {}

  public int getModuleNumber();

  public default void log() {}
}
