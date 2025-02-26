package com.team973.frc2025;

import choreo.util.ChoreoAllianceFlipUtil;
import com.team973.frc2025.auto.modes.DriveTestAuto;
import com.team973.frc2025.auto.modes.LeftSideAuto;
import com.team973.frc2025.auto.modes.NoAuto;
import com.team973.frc2025.auto.modes.NoAutoAllianceWallCenter;
import com.team973.frc2025.auto.modes.RightSideAuto;
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
  private final AutoMode m_testAuto;
  private final AutoMode m_driveTestAuto;
  private final AutoMode m_leftSideAuto;
  private final AutoMode m_rightSideAuto;
  private NoAutoAllianceWallCenter m_noAutoAllianceWallCenter;

  public AutoManager(Logger logger, DriveController drive, Superstructure superstructure) {
    m_noAuto = new NoAuto(logger);
    m_taxiAuto = new TaxiAuto(logger.subLogger("taxi"), drive);
    m_testAuto = new TestAuto(logger.subLogger("test"), drive, superstructure);
    m_driveTestAuto = new DriveTestAuto(logger.subLogger("driveTest"), drive);
    m_leftSideAuto = new LeftSideAuto(logger.subLogger("LeftSideAuto"), superstructure, drive);
    m_rightSideAuto = new RightSideAuto(logger.subLogger("RightSideAuto"), superstructure, drive);
    m_noAutoAllianceWallCenter = new NoAutoAllianceWallCenter(logger);

    m_availableAutos =
        Arrays.asList(
            m_noAuto,
            m_taxiAuto,
            m_testAuto,
            m_driveTestAuto,
            m_noAutoAllianceWallCenter,
            m_leftSideAuto,
            m_rightSideAuto);

    selectAuto(getSelectedMode());
  }

  public void increment() {
    m_selectedMode += 1;
    if (m_selectedMode >= m_availableAutos.size()) {
      m_selectedMode = 0;
    }
    selectAuto(getSelectedMode());
  }

  public void decrement() {
    m_selectedMode -= 1;
    if (m_selectedMode < 0) {
      m_selectedMode = m_availableAutos.size() - 1;
    }
    selectAuto(getSelectedMode());
  }

  public AutoMode getSelectedMode() {
    return m_availableAutos.get(m_selectedMode);
  }

  public Pose2d getStartingPose(Alliance alliance) {
    if (alliance == Alliance.Red) {
      return ChoreoAllianceFlipUtil.flip(m_currentMode.getStartingPose());
    } else {
      return m_currentMode.getStartingPose();
    }
  }

  public void run(Alliance alliance) {
    m_currentMode.run(alliance);
    m_currentMode.log(alliance);
  }

  public void init() {
    selectAuto(m_availableAutos.get(m_selectedMode));
    m_currentMode.init();
  }

  private void selectAuto(AutoMode mode) {
    m_currentMode = mode;
  }
}
