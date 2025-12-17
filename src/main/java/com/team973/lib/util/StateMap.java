package com.team973.lib.util;

import java.util.EnumMap;

public class StateMap<K extends Enum<K>> extends EnumMap<K, SubsystemState> {
  private class EmptySubsystemState implements SubsystemState {
    public void init() {}

    public void run() {}

    public void exit() {}
  }

  public static class Entry<K extends Enum<K>> {
    private final K m_key;
    private final SubsystemState m_state;

    public Entry(K key, SubsystemState state) {
      m_key = key;
      m_state = state;
    }

    public K getKey() {
      return m_key;
    }

    public SubsystemState getState() {
      return m_state;
    }
  }

  public StateMap(Class<K> stateType) {
    super(stateType);

    for (K key : stateType.getEnumConstants()) {
      put(key);
    }
  }

  @SuppressWarnings("unchecked")
  public StateMap(Class<K> stateType, Entry<K>... entries) {
    super(stateType);

    for (Entry<K> e : entries) {
      put(e.getKey(), e.getState());
    }
  }

  private void put(K state) {
    put(state, new EmptySubsystemState());
  }
}
