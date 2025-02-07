package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.*;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingSignaler implements Subsystem {
  private final Logger m_logger;
  private double m_blinkPeriodMs;
  private Color m_colorA;
  private Color m_colorB;
  private CANdle m_candle;

  public BlinkingSignaler(Logger logger, Color colorA, Color colorB, double blinkPeriodMs) {
    m_logger = logger;
    m_colorA = colorA;
    m_colorB = colorB;
    m_blinkPeriodMs = blinkPeriodMs;
  }

  private double modTimeMilisecs() {
    return RobotController.getFPGATime() / 1000.0 % m_blinkPeriodMs;
  }

  public BlinkingSignaler setCandle(CANdle candle) {
    m_candle = candle;
    return this;
  }

  private void blinkingLights() {
    if (modTimeMilisecs() >= 0.5 * m_blinkPeriodMs) {
      m_candle.setLEDs(
          (int) (m_colorA.red * 255.0),
          (int) (m_colorA.green * 255.0),
          (int) (m_colorA.blue * 255.0));
    } else if (modTimeMilisecs() <= 0.5 * m_blinkPeriodMs) {
      m_candle.setLEDs(
          (int) (m_colorB.red * 255.0),
          (int) (m_colorB.green * 255.0),
          (int) (m_colorB.blue * 255.0));
    }
  }

  @Override
  public void log() {
    m_logger.log("current time us", RobotController.getFPGATime());
    SmartDashboard.putNumber("current time secs", RobotController.getFPGATime() / 1000.0 / 1000.0);
    SmartDashboard.putNumber("current time mili mod", modTimeMilisecs());
    SmartDashboard.putNumber("current time mili", RobotController.getFPGATime() / 1000.0);
  }

  @Override
  public void syncSensors() {}

  @Override
  public void update() {
    log();
    blinkingLights();
  }

  @Override
  public void reset() {}
}
