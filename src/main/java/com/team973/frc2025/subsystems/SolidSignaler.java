package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;

public class SolidSignaler implements ISignaler {
  private final Color m_color;

  private boolean m_isEnabled;

  private int m_priortyNum;

  private double m_timeOnMili;

  private double m_timeRequestedMili;

  public SolidSignaler(Color color, double timeMili, int priority) {
    m_color = color;
    m_priortyNum = priority;
    m_timeRequestedMili = timeMili;
    m_timeOnMili = timeMili + RobotController.getFPGATime() / 1000.0;
  }

  private boolean checkTimerUse() {
    if (m_timeRequestedMili == 0) {
      return true;
    } else {
      return false;
    }
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

  @Override
  public void timeOn() {
    if (!checkTimerUse()) {
      if (m_timeOnMili == RobotController.getFPGATime() / 1000.0) {
        setEnabled(false);
      }
    }
  }
}
