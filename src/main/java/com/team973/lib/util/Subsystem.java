package com.team973.lib.util;

/** Base interface for all subsystems */
public abstract class Subsystem<K extends Enum<K>> {
  public abstract static class Stateless extends Subsystem<Stateless.State> {
    private final StateMap<State> m_stateMap = new StateMap<>(State.class);

    public enum State {
      SingleState
    }

    public Stateless() {
      super(State.SingleState);
    }

    public StateMap<State> getStateMap() {
      return m_stateMap;
    }

    public abstract void update();
  }

  private K m_currentState;
  private K m_lastState;

  public Subsystem(K initialState) {
    m_currentState = initialState;
    m_lastState = initialState;
  }

  public abstract StateMap<K> getStateMap();

  public void setState(K state) {
    m_currentState = state;
  }

  public K getState() {
    return m_currentState;
  }

  public void update() {
    if (m_currentState != m_lastState) {
      getStateMap().get(m_lastState).exit();
      getStateMap().get(m_currentState).init();
    }

    getStateMap().get(m_currentState).run();

    m_lastState = m_currentState;
  }

  /** Log subsystem data. Called while the robot is on. */
  public abstract void log();

  /**
   * Update the sensors in the subsystem. This should be called before doing any calculations based
   * on the subsystem.
   */
  public abstract void syncSensors();

  /** Reset the subsystem. */
  public abstract void reset();
}
