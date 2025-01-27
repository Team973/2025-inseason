package com.team973.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;

public class AprilTag {
  private final int m_redID;
  private final int m_blueID;

  private static final HashMap<Integer, Integer> RED_TO_BLUE =
      new HashMap<>() {
        {
          put(-1, -1);
          put(0, 0);
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

  private static final HashMap<Integer, Pose3d> ID_TO_POSE =
      new HashMap<>() {
        {
          put(-1, new Pose3d(0.5, 0, 0, new Rotation3d(0, 0, 0)));
          put(0, new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
          put(1, new Pose3d(16.6972, 0.65532, 1.4859, new Rotation3d(0, 0, Math.toRadians(126))));
          put(2, new Pose3d(16.6972, 7.39648, 1.4859, new Rotation3d(0, 0, Math.toRadians(234))));
          put(3, new Pose3d(11.56081, 8.05561, 1.30175, new Rotation3d(0, 0, Math.toRadians(270))));
          put(4, new Pose3d(9.27608, 6.137656, 1.867916, new Rotation3d(0, Math.toRadians(30), 0)));
          put(5, new Pose3d(9.27608, 1.914906, 1.867916, new Rotation3d(0, Math.toRadians(30), 0)));
          put(
              6,
              new Pose3d(13.47445, 3.306318, 0.308102, new Rotation3d(0, 0, Math.toRadians(300))));
          put(7, new Pose3d(13.8905, 4.0259, 0.308102, new Rotation3d(0, 0, 0)));
          put(
              8,
              new Pose3d(13.47445, 4.745482, 0.308102, new Rotation3d(0, 0, Math.toRadians(60))));
          put(
              9,
              new Pose3d(12.64336, 4.745482, 0.308102, new Rotation3d(0, 0, Math.toRadians(120))));
          put(
              10,
              new Pose3d(12.22731, 4.0259, 0.308102, new Rotation3d(0, 0, Math.toRadians(180))));
          put(
              11,
              new Pose3d(12.64336, 3.306318, 0.308102, new Rotation3d(0, 0, Math.toRadians(240))));
          put(12, new Pose3d(0.851154, 0.65532, 1.4859, new Rotation3d(0, 0, Math.toRadians(54))));
          put(13, new Pose3d(0.851154, 7.39648, 1.4859, new Rotation3d(0, 0, Math.toRadians(306))));
          put(
              14,
              new Pose3d(
                  8.272272,
                  6.137656,
                  1.867916,
                  new Rotation3d(0, Math.toRadians(30), Math.toRadians(180))));
          put(
              15,
              new Pose3d(
                  8.272272,
                  1.914906,
                  1.867916,
                  new Rotation3d(0, Math.toRadians(30), Math.toRadians(180))));
          put(
              16,
              new Pose3d(5.987542, -0.00381, 1.30175, new Rotation3d(0, 0, Math.toRadians(90))));
          put(
              17,
              new Pose3d(4.073906, 3.306318, 0.308102, new Rotation3d(0, 0, Math.toRadians(240))));
          put(18, new Pose3d(3.6576, 4.0259, 0.308102, new Rotation3d(0, 0, Math.toRadians(180))));
          put(
              19,
              new Pose3d(4.073906, 4.745482, 0.308102, new Rotation3d(0, 0, Math.toRadians(120))));
          put(
              20,
              new Pose3d(4.90474, 4.745482, 0.308102, new Rotation3d(0, 0, Math.toRadians(60))));
          put(21, new Pose3d(5.321046, 4.0259, 0.308102, new Rotation3d(0, 0, 0)));
          put(
              22,
              new Pose3d(4.90474, 3.306318, 0.308102, new Rotation3d(0, 0, Math.toRadians(300))));
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

  public Pose3d getPoseFromAlliance(Alliance alliance) {
    return ID_TO_POSE.get(getIDFromAlliance(alliance));
  }
}
