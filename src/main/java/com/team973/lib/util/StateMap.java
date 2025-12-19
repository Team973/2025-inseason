package com.team973.lib.util;

import java.util.EnumMap;

public class StateMap<K extends Enum<K>> {
  private final EnumMap<K, SubsystemState> m_enumMap;

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
    m_enumMap = new EnumMap<>(stateType);

    for (K key : stateType.getEnumConstants()) {
      put(key);
    }
  }

  @SuppressWarnings("unchecked")
  public StateMap(Class<K> stateType, Entry<K>... entries) {
    m_enumMap = new EnumMap<>(stateType);

    for (Entry<K> e : entries) {
      m_enumMap.put(e.getKey(), e.getState());
    }
  }

  private void put(K state) {
    m_enumMap.put(state, new EmptySubsystemState());
  }

  public SubsystemState get(K key) {
    return m_enumMap.get(key);
  }
}
