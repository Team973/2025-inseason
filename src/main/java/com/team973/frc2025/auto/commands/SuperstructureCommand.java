package com.team973.frc2025.auto.commands;

import com.team973.frc2025.subsystems.Superstructure;
import com.team973.lib.util.AutoCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SuperstructureCommand extends AutoCommand {
  private final Superstructure m_superstructure;
  private final Superstructure.State m_state;

  public SuperstructureCommand(Superstructure superstructure, Superstructure.State state) {
    m_superstructure = superstructure;
    m_state = state;
  }

  @Override
  public void init() {
    m_superstructure.setState(m_state);
  }

  @Override
  public void run(Alliance alliance) {}

  @Override
  public void log(Alliance alliance) {}

  @Override
  public boolean isCompleted() {
    return true;
  }

  @Override
  public void postComplete(boolean interrupted) {}
}
