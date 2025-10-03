package com.team973.frc2025;

import com.team973.frc2025.shared.RobotInfo;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.error.MarkedYAMLException;
import org.yaml.snakeyaml.error.YAMLException;

public class RobotConfig {

  private static boolean m_initialized;
  private static RobotInfo m_robotInfo;

  public static synchronized RobotInfo get() {
    if (!m_initialized) {
      loadRobotInfo();
    }
    return m_robotInfo;
  }

  private static void loadRobotInfo() {
    String filePath = Filesystem.getDeployDirectory().toPath().resolve("Config.yaml").toString();

    try {
      // Load YAML file into a string
      String yamlContent = new String(Files.readAllBytes(Paths.get(filePath)), "UTF-8");
      Yaml yaml = new Yaml();
      m_robotInfo = yaml.loadAs(yamlContent, RobotInfo.class);
      m_initialized = true;

    } catch (MarkedYAMLException e) {
      System.err.println("YAML parsing error:");
      System.err.println("  " + e.getMessage());
      if (e.getProblemMark() != null) {
        System.err.println(
            "  at line "
                + (e.getProblemMark().getLine() + 1)
                + ", column "
                + (e.getProblemMark().getColumn() + 1));
      }
      throw e; // rethrow if you want program to stop

    } catch (YAMLException e) {
      System.err.println("General YAML parsing error: " + e.getMessage());
      throw e;

    } catch (IOException e) {
      System.err.println("Could not read file: " + e.getMessage());
      throw new RuntimeException(e);
    }
  }
}
