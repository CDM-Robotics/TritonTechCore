package org.tritontech.core;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class MotorFactory {

    static {
        VersionManager.initialize(); // Triggers VersionManager's static block
    }

    // Pass in the type of motor controller (SPARK_MAX or SPARK_FLEX)
    public static SparkBase createMotor(MotorControllerType type, int deviceId, MotorType motorType) {
        switch (type) {
            case SPARK_MAX -> {
                return new SparkMax(deviceId, motorType);
            }
            case SPARK_FLEX -> {
                return new SparkFlex(deviceId, motorType);
            }
            default -> throw new IllegalArgumentException("Unsupported motor type: " + type);
        }
    }
}