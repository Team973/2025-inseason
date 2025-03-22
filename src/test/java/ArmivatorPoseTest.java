import static org.junit.Assert.assertEquals;

import com.team973.frc2025.shared.RobotInfo.ArmInfo;
import com.team973.frc2025.subsystems.Superstructure.ArmivatorPose;
import org.junit.jupiter.api.Test;

public class ArmivatorPoseTest {
  @Test
  void testOne() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(0.3, 0.2);

    System.out.println(pose.getArmAngle());
    System.out.println(pose.getElevatorHeight());

    assertEquals(-48.315, pose.getArmAngle(), 0.01);
    assertEquals(0.537, pose.getElevatorHeight(), 0.01);
  }

  @Test
  void testTwo() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(0.4, 0.6);

    System.out.println(pose.getArmAngle());
    System.out.println(pose.getElevatorHeight());

    assertEquals(27.537, pose.getArmAngle(), 0.01);
    assertEquals(0.391, pose.getElevatorHeight(), 0.01);
  }

  @Test
  void testThree() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(0.05, 1.15);

    System.out.println(pose.getArmAngle());
    System.out.println(pose.getElevatorHeight());

    assertEquals(ArmInfo.ARM_MAX_ANGLE_DEG, pose.getArmAngle(), 0.01);
    assertEquals(0.702, pose.getElevatorHeight(), 0.01);
  }

  @Test
  void testFour() throws Exception {
    ArmivatorPose pose = ArmivatorPose.fromCoordinate(20.0, 0.6);

    System.out.println(pose.getArmAngle());
    System.out.println(pose.getElevatorHeight());

    assertEquals(0.0, pose.getArmAngle(), 0.01);
    assertEquals(0.6, pose.getElevatorHeight(), 0.01);
  }
}
