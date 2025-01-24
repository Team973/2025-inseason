package com.team973.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class DriveComposable {
  private boolean m_firstRun = true;

  public static class Config {
    public enum Composable {
      A,
      B
    }

    public Composable translation = Composable.A;
    public Composable rotation = Composable.A;
  }

  public static DriveComposable compose(
      DriveComposable composableA, DriveComposable composableB, Config config) {
    return new DriveComposable() {
      @Override
      public ChassisSpeeds getOutput() {
        ChassisSpeeds output = new ChassisSpeeds();

        ChassisSpeeds outputA = composableA.getOutput();
        ChassisSpeeds outputB = composableB.getOutput();

        if (config.translation == Config.Composable.A) {
          output.vxMetersPerSecond = outputA.vxMetersPerSecond;
          output.vyMetersPerSecond = outputA.vyMetersPerSecond;
        } else {
          output.vxMetersPerSecond = outputB.vxMetersPerSecond;
          output.vyMetersPerSecond = outputB.vyMetersPerSecond;
        }

        if (config.rotation == Config.Composable.A) {
          output.omegaRadiansPerSecond = outputA.omegaRadiansPerSecond;
        } else {
          output.omegaRadiansPerSecond = outputB.omegaRadiansPerSecond;
        }

        return output;
      }
    };
  }

  public void init() {
    m_firstRun = true;
  }

  public void firstRunComplete() {
    m_firstRun = false;
  }

  public boolean getFirstRun() {
    return m_firstRun;
  }

  public abstract ChassisSpeeds getOutput();
}
