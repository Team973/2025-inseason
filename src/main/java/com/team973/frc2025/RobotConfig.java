package com.team973.frc2025;

import com.team973.frc2025.shared.RobotInfo;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.yaml.snakeyaml.Yaml;

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
    String filePath = System.getProperty("user.dir") + "/config/robot-info-var/Config.yaml";

    try {
      // Load YAML file into a string
      String yamlContent = new String(Files.readAllBytes(Paths.get(filePath)), "UTF-8");
      Yaml yaml = new Yaml();
      m_robotInfo = yaml.loadAs(yamlContent, RobotInfo.class);
      m_initialized = true;

    } catch (IOException e) {
      System.out.println("⚠️ Could not load config file, using defaults");
      System.exit(1);
    }
  }
}
