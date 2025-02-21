package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.team973.lib.util.Conversions;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingSignaler implements ISignaler {

  private double m_blinkPeriodMs;
  public int m_priorty;

  private boolean m_isEnabled;

  private Color m_colorA;
  private Color m_colorB;

  private double m_timeRequestedMili;
  private double m_enabledStartTime;

  public BlinkingSignaler(
      Color colorA, Color colorB, double blinkPeriodMs, double timeOnMili, int priorityNum) {
    m_colorA = colorA;
    m_colorB = colorB;
    m_blinkPeriodMs = blinkPeriodMs;
    m_priorty = priorityNum;
    m_timeRequestedMili = timeOnMili;
  }

  private double modTimeMilisecs() {
    return Conversions.Time.getMsecTime() % m_blinkPeriodMs;
  }

  private void blinkingLights(CANdle candle) {
    if (modTimeMilisecs() >= 0.5 * m_blinkPeriodMs) {
      candle.setLEDs(
          (int) (m_colorA.red * 255.0),
          (int) (m_colorA.green * 255.0),
          (int) (m_colorA.blue * 255.0));
    } else if (modTimeMilisecs() <= 0.5 * m_blinkPeriodMs) {
      candle.setLEDs(
          (int) (m_colorB.red * 255.0),
          (int) (m_colorB.green * 255.0),
          (int) (m_colorB.blue * 255.0));
    }
  }

  public void log(ISignaler signaler) {}

  public void update(CANdle candle) {
    blinkingLights(candle);
  }

  public boolean isInfiniteTime() {
    return m_timeRequestedMili == 0;
  }

  public boolean isEnabledTimer() {
    return Conversions.Time.getMsecTime() < m_enabledStartTime + m_timeRequestedMili;
  }

  @Override
  public boolean isEnabled() {
    return m_isEnabled && (isInfiniteTime() || isEnabledTimer());
  }

  @Override
  public int getPriorty() {
    return m_priorty;
  }

  @Override
  public void enable() {
    m_isEnabled = true;
    m_enabledStartTime = Conversions.Time.getMsecTime();
  }

  @Override
  public void disable() {
    m_isEnabled = false;
  }
}
