import static org.junit.Assert.assertEquals;

import com.team973.lib.util.AprilTag;
import com.team973.lib.util.TargetPositionRelativeToAprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class TargetPositionRelativeToAprilTagTest {
  @Test
  void testOne() throws Exception {
    TargetPositionRelativeToAprilTag target =
        new TargetPositionRelativeToAprilTag(
            AprilTag.fromRed(1), new Translation2d(1, 1), 0.5, Rotation2d.fromDegrees(0));
    System.out.println("Target Initial X: " + target.getApproachPose().getX());
    System.out.println("Target Initial Y: " + target.getApproachPose().getY());
    System.out.println(
        "Target Initial Deg: " + target.getApproachPose().getRotation().getDegrees());

    assertEquals(target.getApproachPose().getX(), 15.3004, 0.001);
    assertEquals(target.getApproachPose().getY(), 0.8766, 0.001);
  }
}
