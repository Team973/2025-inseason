package com.team973.lib.util;

import java.util.EnumMap;

public class StateMap<K extends Enum<K>> extends EnumMap<K, SubsystemState> {
  private class EmptySubsystemState implements SubsystemState {
    public void init() {}

    public void run() {}

    public void exit() {}
  }

  public StateMap(Class<K> stateType) {
    super(stateType);
  }

  public void put(K state) {
    put(state, new EmptySubsystemState());
  }
}
