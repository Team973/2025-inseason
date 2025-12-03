package com.team973.frc2025;

import com.fasterxml.jackson.databind.*;
import com.team973.frc2025.shared.RobotInfo;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class RobotConfig {

  private static boolean m_initialized;
  private static RobotInfo m_robotInfo;

  public static enum ConfigSource {
    Common, // default options for all robots.

    // TODO: It would be nice to have overrides for red vs blue alliance
    // but we need this data at init time and we don't get the alliance
    // until the robot connects to the FMS.

    Simulation, // overrides for when running in simulation
    Real, // overrides for the real robot
  }

  public static synchronized RobotInfo get() {
    if (!m_initialized) {
      List<ConfigSource> configSources = getConfigSources();
      System.err.printf("Using configSources: %s\n", configSources);
      m_robotInfo = getMergedConfig(getConfigSources());
      m_initialized = true;
    }
    return m_robotInfo;
  }

  /** configSuffix is something like "common", "simulation", */
  private static String absPathForConfigSource(ConfigSource configSource) {
    return Filesystem.getDeployDirectory()
        .toPath()
        .resolve("config-" + configSource.toString().toLowerCase() + ".json")
        .toString();
  }

  public static List<ConfigSource> getConfigSources() {
    List<ConfigSource> res = new ArrayList<>();
    res.add(ConfigSource.Common);

    if (RobotBase.isSimulation()) {
      res.add(ConfigSource.Simulation);
    } else {
      res.add(ConfigSource.Real);
    }

    return res;
  }

  public static RobotInfo getMergedConfig(List<ConfigSource> configSources) {
    RobotInfo res = new RobotInfo();

    for (ConfigSource src : configSources) {

      ObjectMapper mapper = new ObjectMapper();
      mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, true);
      mapper.setDefaultMergeable(true);
      try {
        mapper.readerForUpdating(res).readValue(new File(absPathForConfigSource(src)));
      } catch (com.fasterxml.jackson.core.JsonParseException e) {
        System.err.printf("YAML parsing error: %s\n", e.getMessage());
        throw new RuntimeException(e);
      } catch (IOException e) {
        System.err.println("Could not read file: " + e.getMessage());
        throw new RuntimeException(e);
      }
    }

    return res;
  }
}
