package com.team973.frc2025.subsystems;

import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.util.Subsystem;

public interface ElevatorIO extends Subsystem {
  public void setHallZeroingEnabled(boolean zeroingMode);

  public void home();

  public void setControlStatus(Elevator.ControlStatus status);

  public boolean motorAtTarget();

  public void setTargetPostion(double targetPostionHeightinches);

  public void incrementOffset(double offset, ReefLevel level);

  public double getTargetPosition();

  public double getTargetPositionFromLevel(ReefLevel level);

  public double getHeightInchesFromMotorRot(double motorRot);
}
