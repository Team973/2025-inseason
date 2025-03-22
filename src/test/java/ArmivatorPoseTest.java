import static org.junit.Assert.assertEquals;

import com.team973.frc2025.subsystems.Superstructure.ArmivatorPose;
import org.junit.jupiter.api.Test;

public class ArmivatorPoseTest {
  private static final ArmivatorPose.Config ARMIVATOR_TEST_CONFIG =
      new ArmivatorPose.Config(0.451104, 75.0, -80.0, 0.7112);

  @Test
  void testArmAboveTarget() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(0.3, 0.2, ARMIVATOR_TEST_CONFIG);

    System.out.println(pose.toString());

    assertEquals(false, pose.getTargetIsOutOfBounds());
    assertEquals(-48.315, pose.getArmAngleDeg(), 0.01);
    assertEquals(0.537, pose.getElevatorHeightMeters(), 0.01);
  }

  @Test
  void testArmBelowTarget() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(0.4, 0.6, ARMIVATOR_TEST_CONFIG);

    System.out.println(pose.toString());

    assertEquals(false, pose.getTargetIsOutOfBounds());
    assertEquals(27.537, pose.getArmAngleDeg(), 0.01);
    assertEquals(0.391, pose.getElevatorHeightMeters(), 0.01);
  }

  @Test
  void testArmAngleClamp() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(0.05, 1.15, ARMIVATOR_TEST_CONFIG);

    System.out.println(pose.toString());

    assertEquals(true, pose.getTargetIsOutOfBounds());
    assertEquals(ARMIVATOR_TEST_CONFIG.maxArmAngleDeg, pose.getArmAngleDeg(), 0.01);
    assertEquals(0.702, pose.getElevatorHeightMeters(), 0.01);
  }

  @Test
  void testXClamp() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(20.0, 0.6, ARMIVATOR_TEST_CONFIG);

    System.out.println(pose.toString());

    assertEquals(false, pose.getTargetIsOutOfBounds());
    assertEquals(0.0, pose.getArmAngleDeg(), 0.01);
    assertEquals(0.6, pose.getElevatorHeightMeters(), 0.01);
  }
}
