package com.team973.lib.util;

import com.team973.frc2025.subsystems.Superstructure;

public abstract class State {
  private final Superstructure m_superstructure;

  public State(Superstructure superstructure) {
    m_superstructure = superstructure;
  }

  protected Superstructure getSuperstructure() {
    return m_superstructure;
  }

  public abstract void onEntrance();

  public abstract void run();

  public abstract void onExit();
}
