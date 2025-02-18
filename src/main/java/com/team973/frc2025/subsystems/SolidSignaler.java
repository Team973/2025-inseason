package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.team973.lib.util.Conversions;
import edu.wpi.first.wpilibj.util.Color;

public class SolidSignaler implements ISignaler {
  private final Color m_color;

  private boolean m_isEnabled;

  private int m_priortyNum;

  private double m_timeRequestedMili;

  private double m_enabledStartTime;

  public SolidSignaler(Color color, double timeMili, int priority) {
    m_color = color;
    m_priortyNum = priority;
    m_timeRequestedMili = timeMili;
  }

  private boolean isInfiniteTime() {
    return m_timeRequestedMili == 0;
  }

  private boolean isEnabledTimer() {
    return Conversions.Time.getMsecTime() > m_enabledStartTime + m_timeRequestedMili;
  }

  @Override
  public void update(CANdle candle) {
    candle.setLEDs(
        (int) (m_color.red * 255.0), (int) (m_color.green * 255.0), (int) (m_color.blue * 255.0));
  }

  @Override
  public boolean isEnabled() {
    return m_isEnabled && (isInfiniteTime() || isEnabledTimer());
  }

  @Override
  public int getPriorty() {
    return m_priortyNum;
  }

  @Override
  public void enable() {
    m_isEnabled = true;
    m_enabledStartTime = Conversions.Time.getMsecTime();
  }

  public void disable() {
    m_isEnabled = false;
  }
}
