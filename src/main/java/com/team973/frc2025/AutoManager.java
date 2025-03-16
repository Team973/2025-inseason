package com.team973.frc2025;

import choreo.util.ChoreoAllianceFlipUtil;
import com.team973.frc2025.auto.modes.BabyBird;
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
  private final List<AutoMode> m_availableAutos;
  private int m_selectedMode = 0;

  private final AutoMode m_noAuto;
  private final AutoMode m_taxiAuto;
  private final AutoMode m_testAuto;
  private final AutoMode m_driveTestAuto;
  private final AutoMode m_leftSideAuto;
  private final AutoMode m_leftSideBabybirdAuto;
  private final AutoMode m_rightSideAuto;
  private final AutoMode m_rightSideBabybirdAuto;
  private final AutoMode m_noAutoAllianceWallCenter;
  private final AutoMode m_babyBirdAuto;

  public AutoManager(Logger logger, DriveController drive, Superstructure superstructure) {
    m_noAuto = new NoAuto(logger);
    m_taxiAuto = new TaxiAuto(logger.subLogger("taxi"), drive);
    m_testAuto = new TestAuto(logger.subLogger("test"), drive, superstructure);
    m_driveTestAuto = new DriveTestAuto(logger.subLogger("driveTest"), drive);
    m_leftSideAuto =
        new LeftSideAuto(logger.subLogger("LeftSideAuto"), superstructure, drive, false);
    m_leftSideBabybirdAuto =
        new LeftSideAuto(logger.subLogger("LeftSideBabybirdAuto"), superstructure, drive, true);
    m_rightSideAuto =
        new RightSideAuto(logger.subLogger("RightSideAuto"), superstructure, drive, false);
    m_rightSideBabybirdAuto =
        new RightSideAuto(logger.subLogger("RightSideBabybirdAuto"), superstructure, drive, true);
    m_noAutoAllianceWallCenter = new NoAutoAllianceWallCenter(logger);
    m_babyBirdAuto = new BabyBird(logger, drive, superstructure);

    m_availableAutos =
        Arrays.asList(
            m_noAuto,
            m_taxiAuto,
            m_testAuto,
            m_driveTestAuto,
            m_noAutoAllianceWallCenter,
            m_leftSideAuto,
            m_leftSideBabybirdAuto,
            m_rightSideAuto,
            m_rightSideBabybirdAuto,
            m_babyBirdAuto);
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

  public Pose2d getStartingPose(Alliance alliance) {
    if (alliance == Alliance.Red) {
      return ChoreoAllianceFlipUtil.flip(getSelectedMode().getStartingPose());
    } else {
      return getSelectedMode().getStartingPose();
    }
  }

  public void run(Alliance alliance) {
    getSelectedMode().run(alliance);
    getSelectedMode().log(alliance);
  }

  public void init() {
    getSelectedMode().init();
  }
}
