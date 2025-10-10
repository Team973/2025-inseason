import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import org.junit.jupiter.api.Test;
import org.yaml.snakeyaml.Yaml;

public class ConfigFileTest {
  @Test
  void testGetConfig() throws Exception {
    RobotInfo robotInfo = RobotConfig.get();
    System.out.printf("Config: %s\n", (new Yaml().dump(robotInfo)));
    System.out.printf("ELEVATOR_V_KP is %f\n", robotInfo.ELEVATOR_INFO.ELEVATOR_V_KP);
    assertEquals(125.0, robotInfo.ELEVATOR_INFO.ELEVATOR_V_KP, 0.01);
  }
}
