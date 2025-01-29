package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.team973.lib.util.Logger;
import com.team973.lib.util.Subsystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANdleManger implements Subsystem {
  private final Logger m_logger;
  private final CANdle m_candle = new CANdle(18, "rio");
  private double m_mod = 0;

  public CANdleManger(Logger logger) {
    m_logger = logger;
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  //   public static int flashPerMiliSec(int flashesPerMiliSec){

  //   }

  private double modTimeMilisecs() {
    return RobotController.getFPGATime() / 1000.0 % m_mod;
  }

  private void bliningLights() {
    if (modTimeMilisecs() >= 500) {
      m_candle.setLEDs(0, 0, 0);
    } else if (modTimeMilisecs() <= 499) {
      m_candle.setLEDs(255, 255, 200);
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
    bliningLights();
  }

  @Override
  public void reset() {}
}
