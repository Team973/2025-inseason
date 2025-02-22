package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color;

public class SolidSignaler implements ISignaler {
  private final Color m_color;

  private boolean m_isEnabled;

  private int m_priortyNum;

  public SolidSignaler(Color color, int priority) {
    m_color = color;
    m_priortyNum = priority;
  }

  @Override
  public void update(CANdle candle) {
    candle.setLEDs(
        (int) (m_color.red * 255.0), (int) (m_color.green * 255.0), (int) (m_color.blue * 255.0));
  }

  @Override
  public boolean isEnabled() {
    return m_isEnabled;
  }

  @Override
  public int getPriorty() {
    return m_priortyNum;
  }

  @Override
  public void setEnabled(boolean enabled) {
    m_isEnabled = enabled;
  }
}
