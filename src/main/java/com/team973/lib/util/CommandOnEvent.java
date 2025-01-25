package com.team973.lib.util;

public class CommandOnEvent {
  private final String m_eventName;
  private final AutoCommand m_command;

  public CommandOnEvent(String eventName, AutoCommand command) {
    m_eventName = eventName;
    m_command = command;
  }

  public String getEventName() {
    return m_eventName;
  }

  public AutoCommand getCommand() {
    return m_command;
  }
}
