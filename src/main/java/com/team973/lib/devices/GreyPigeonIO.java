package com.team973.lib.devices;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.ArrayList;

public interface GreyPigeonIO {
  public void simulationUpdate();

  public ArrayList<StatusSignal<Angle>> getAngleStatusSignals();

  public ArrayList<StatusSignal<AngularVelocity>> getAngularVelocityStatusSignals();

  public Rotation2d getYaw();

  public Rotation2d getRawYaw();

  public Rotation2d getAngularVelocity();

  public void setYawOffset(Rotation2d newYaw);

  public void reset();

  public void log();
}
