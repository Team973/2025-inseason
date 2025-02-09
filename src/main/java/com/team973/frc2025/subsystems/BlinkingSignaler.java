package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.team973.lib.util.Logger;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingSignaler implements ISignaler {
  private final Logger m_logger;

  private double m_blinkPeriodMs;

  private boolean m_isEnabled;

  private Color m_colorA;
  private Color m_colorB;
  private Color m_currentColor;

  public BlinkingSignaler(Logger logger, Color colorA, Color colorB, double blinkPeriodMs) {
    m_logger = logger;
    m_colorA = colorA;
    m_colorB = colorB;
    m_blinkPeriodMs = blinkPeriodMs;
  }

  private double modTimeMilisecs() {
    return RobotController.getFPGATime() / 1000.0 % m_blinkPeriodMs;
  }

  public Color getCurrentColor() {
    return m_currentColor;
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

  public void log() {

    m_logger.log("current time us", RobotController.getFPGATime());
    SmartDashboard.putNumber("current time secs", RobotController.getFPGATime() / 1000.0 / 1000.0);
    SmartDashboard.putNumber("current time mili mod", modTimeMilisecs());
    SmartDashboard.putNumber("current time mili", RobotController.getFPGATime() / 1000.0);
    SmartDashboard.putNumber("blink time MOD", m_blinkPeriodMs);
    SmartDashboard.putString("colorA", m_colorA.toString());
    SmartDashboard.putString("colorB", m_colorB.toString());
  }

  public void syncSensors() {}

  public void update(CANdle candle) {
    log();
    blinkingLights(candle);
  }

  public void reset() {}

  @Override
  public boolean isEnabled() {
    return m_isEnabled;
  }

  @Override
  public void setEnabled(boolean enabled) {
    m_isEnabled = enabled;
  }
}
