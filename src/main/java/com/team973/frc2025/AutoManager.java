package com.team973.frc2025;

import com.team973.frc2025.auto.modes.BabyBird;
import com.team973.frc2025.auto.modes.DriveTestAuto;
import com.team973.frc2025.auto.modes.NoAuto;
import com.team973.frc2025.auto.modes.NoAutoAllianceWallCenter;
import com.team973.frc2025.auto.modes.TaxiAuto;
import com.team973.frc2025.auto.modes.TestAuto;
import com.team973.frc2025.subsystems.DriveController;
import com.team973.frc2025.subsystems.Superstructure;
import com.team973.lib.util.AutoMode;
import com.team973.lib.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import java.util.List;

public class AutoManager {
  private AutoMode m_currentMode;
  private final List<AutoMode> m_availableAutos;
  private int m_selectedMode = 0;

  private final AutoMode m_noAuto;
  private final AutoMode m_taxiAuto;
  private final AutoMode m_BabyBirdAuto;
  private final AutoMode m_testAuto;
  private final AutoMode m_driveTestAuto;
  private NoAutoAllianceWallCenter m_noAutoAllianceWallCenter;

  public AutoManager(Logger logger, DriveController drive, Superstructure superstructure) {
    m_noAuto = new NoAuto(logger);
    m_BabyBirdAuto = new BabyBird(logger.subLogger("BabyBird"), drive);
    m_taxiAuto = new TaxiAuto(logger.subLogger("taxi"), drive);
    m_testAuto = new TestAuto(logger.subLogger("test"), drive, superstructure);
    m_driveTestAuto = new DriveTestAuto(logger.subLogger("driveTest"), drive);
    m_noAutoAllianceWallCenter = new NoAutoAllianceWallCenter(logger);

    m_availableAutos =
        Arrays.asList(
            m_noAuto,
            m_taxiAuto,
            m_BabyBirdAuto,
            m_testAuto,
            m_driveTestAuto,
            m_noAutoAllianceWallCenter);
  }

  public void increment() {
    m_selectedMode += 1;
    if (m_selectedMode >= m_availableAutos.size()) {
      m_selectedMode = 0;
    }
  }

  public void decrement() {
    m_selectedMode -= 1;
    if (m_selectedMode < 0) {
      m_selectedMode = m_availableAutos.size() - 1;
    }
  }

  public AutoMode getSelectedMode() {
    return m_availableAutos.get(m_selectedMode);
  }

  // TODO: Should account for alliance here and not in disabledPeriodic.
  // TODO: Need to account for both position and rotation (currently we only
  // account for rotation).
  public Pose2d getStartingPose(Alliance alliance) {
    return m_currentMode.getStartingPose(alliance);
  }

  public void run() {
    m_currentMode.run();
    m_currentMode.log();
  }

  public void init() {
    selectAuto(m_availableAutos.get(m_selectedMode));
    m_currentMode.init();
  }

  private void selectAuto(AutoMode mode) {
    m_currentMode = mode;
  }
}
