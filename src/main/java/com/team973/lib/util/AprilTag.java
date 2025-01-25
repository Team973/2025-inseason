package com.team973.lib.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;

public class AprilTag {
  private final int m_redID;
  private final int m_blueID;

  private static final HashMap<Integer, Integer> RED_TO_BLUE =
      new HashMap<>() {
        {
          put(1, 13);
          put(2, 12);
          put(3, 16);
          put(4, 15);
          put(5, 14);
          put(6, 19);
          put(7, 18);
          put(8, 17);
          put(9, 22);
          put(10, 21);
          put(11, 20);
        }
      };

  public AprilTag(int redID, int blueID) {
    m_redID = redID;
    m_blueID = blueID;
  }

  public static AprilTag fromRed(int redID) {
    return new AprilTag(redID, RED_TO_BLUE.get(redID));
  }

  public int getIDFromAlliance(Alliance alliance) {
    return alliance == Alliance.Red ? m_redID : m_blueID;
  }
}
