package com.team973.frc2025;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import org.yaml.snakeyaml.Yaml;

public class RobotConfig {
    public interface ConfigSupplier<T> {
        T get(RobotConfig config);
    }

    private final Map<String, Object> m_data = new HashMap<>();

    public RobotConfig() {
        String filePath = System.getProperty("user.dir") + "/config/robot-info-var/Config.yaml";

        try {
            // Load YAML file into a string
            String yamlContent = new String(Files.readAllBytes(Paths.get(filePath)), "UTF-8");
            Yaml yaml = new Yaml();
            Map<String, Object> loaded = yaml.load(yamlContent);

            if (loaded != null) {
                m_data.putAll(loaded);
            }
        } catch (IOException e) {
            System.out.println("⚠️ Could not load config file, using defaults");
            loadDefaults();
        }
    }

    /** Fallback defaults if YAML can't be loaded */
    private void loadDefaults() {
        Map<String, Object> armInfo = new HashMap<>();
        armInfo.put("HALL_SENSOR_ID", 0);
        armInfo.put("ENCODER_CAN_ID", 1);
        armInfo.put("ENCODER_OFFSET_ROTATIONS", 0.0);
        m_data.put("ArmInfo", armInfo);

        Map<String, Object> server = new HashMap<>();
        server.put("host", "localhost");
        server.put("port", 8080);
        m_data.put("Server", server);
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> getSection(String key) {
        return (Map<String, Object>) m_data.getOrDefault(key, new HashMap<>());
    }

    public int getInt(String parentKey, String childKey, int defaultValue) {
        Object value = getSection(parentKey).get(childKey);
        return (value instanceof Number) ? ((Number) value).intValue() : defaultValue;
    }

    public double getDouble(String parentKey, String childKey, double defaultValue) {
        Object value = getSection(parentKey).get(childKey);
        return (value instanceof Number) ? ((Number) value).doubleValue() : defaultValue;
    }

    public String getString(String parentKey, String childKey, String defaultValue) {
        Object value = getSection(parentKey).get(childKey);
        return (value instanceof String) ? (String) value : defaultValue;
    }

    /** Call a ConfigSupplier safely */
    public <T> T use(ConfigSupplier<T> supplier) {
        return supplier.get(this);
    }
}
