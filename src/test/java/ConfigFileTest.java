import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team973.frc2025.shared.RobotInfo;
import org.junit.jupiter.api.Test;
import org.yaml.snakeyaml.Yaml;

public class ConfigFileTest {
  @Test
  void testGetConfig() throws Exception {
    RobotInfo robotInfo = new RobotInfo();
    System.out.printf("Config: %s\n", (new Yaml().dump(robotInfo)));
    System.out.printf("ANGLE_GEAR_RATIO is %f\n", robotInfo.DRIVE_INFO.ANGLE_GEAR_RATIO);
    assertEquals(15.0004, robotInfo.DRIVE_INFO.ANGLE_GEAR_RATIO, 0.01);
  }
}
