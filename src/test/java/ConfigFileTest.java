import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team973.frc2025.RobotConfig;
import com.team973.frc2025.shared.RobotInfo;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.yaml.snakeyaml.Yaml;

public class ConfigFileTest {
  /**
   * Get the config using the same codepath that the robot will use (RobotConfig.get()).
   *
   * <p>We expect that tests are always running in simulation mode so we will also test that the
   * swerve module offsets are 0 (since they should always be zero in simulation).
   */
  @Test
  void testGetConfig() throws Exception {
    RobotInfo robotInfo = RobotConfig.get();
    System.out.printf("Config: %s\n", (new Yaml().dump(robotInfo)));
    System.out.printf("ELEVATOR_V_KP is %f\n", robotInfo.ELEVATOR_INFO.ELEVATOR_V_KP);
    assertEquals(125.0, robotInfo.ELEVATOR_INFO.ELEVATOR_V_KP, 0.01);
    assertEquals(0.0, robotInfo.DRIVE_INFO.BACK_LEFT_MODULE_STEER_OFFSET, 0.001);
  }

  /**
   * This test really just checks to see that all the config files can be parsed. There's no
   * scenario IRL where we would merge _every_ config file because some of them are mutually
   * exclusive.
   */
  @Test
  void testAllConfigPermutations() throws Exception {
    List<RobotConfig.ConfigSource> allConfigSources = List.of(RobotConfig.ConfigSource.values());
    RobotConfig.getMergedConfig(allConfigSources);
  }
}
