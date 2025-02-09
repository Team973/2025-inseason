package com.team973.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;

public interface ISignaler {
  public boolean isEnabled();

  public void setEnabled(boolean enabled);

  public void update(CANdle candle);
}
