package com.team973.frc2025.subsystems;

import com.team973.frc2025.subsystems.Superstructure.ReefLevel;
import com.team973.lib.util.StateMap;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.math.geometry.Pose3d;

public abstract class ElevatorIO extends Subsystem<ElevatorIO.State> {
  private final StateMap<State> m_stateMap;

  public enum State {
    TargetPostion,
    Zero,
    Off
  }

  public ElevatorIO() {
    super(State.Off);

    m_stateMap = new StateMap<>(State.class);

    m_stateMap.put(State.TargetPostion);
    m_stateMap.put(State.Zero);
    m_stateMap.put(State.Off);
  }

  public StateMap<State> getStateMap() {
    return m_stateMap;
  }

  public abstract void setHallZeroingEnabled(boolean zeroingMode);

  public abstract void home();

  public abstract boolean motorAtTarget();

  public abstract void setTargetPostion(double targetPostionHeightinches);

  public abstract void incrementOffset(double offset, ReefLevel level);

  public abstract double getTargetPosition();

  public abstract double getTargetPositionFromLevel(ReefLevel level);

  public abstract double getHeightInchesFromMotorRot(double motorRot);

  public abstract Pose3d getPose();
}
