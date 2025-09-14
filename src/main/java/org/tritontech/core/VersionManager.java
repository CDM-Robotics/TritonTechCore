package org.tritontech.core;

import java.io.IOException;
import java.util.Properties;

public class VersionManager {
    private static final String VERSION;
    private static boolean hasPrinted = false;

    static {
        // Load version from properties file
        String version = "unknown";
        try {
            Properties props = new Properties();
            props.load(VersionManager.class.getResourceAsStream("/org/tritontech/core/version.properties"));
            version = props.getProperty("version", "unknown");
        } catch (IOException e) {
            System.err.println("Failed to load version.properties: " + e.getMessage());
        }
        VERSION = version;

        synchronized (VersionManager.class) {
            if (!hasPrinted) {
                System.out.println("TritonTechCore Library Version: " + VERSION);
                hasPrinted = true;
            }
        }
    }

    // Private constructor to prevent instantiation
    private VersionManager() {
        throw new AssertionError("Utility class, cannot be instantiated");
    }

    public static String getVersion() {
        return VERSION;
    }

    public static void initialize() {
        // Empty method, triggers static block
    }
}