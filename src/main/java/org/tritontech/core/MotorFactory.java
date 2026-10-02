package org.tritontech.core;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import org.wpilib.hardware.bus.CANPort;

public class MotorFactory {

    static {
        VersionManager.initialize(); // Triggers VersionManager's static block
    }

    /** CAN bus used by the overloads that don't take one. */
    public static final CANPort DEFAULT_CAN_BUS = CANPort.CAN_S0;

    // Pass in the type of motor controller (SPARK_MAX or SPARK_FLEX)
    public static SparkBase createMotor(MotorControllerType type, int deviceId, MotorType motorType) {
        return createMotor(type, DEFAULT_CAN_BUS, deviceId, motorType);
    }

    // SystemCore has multiple CAN buses, so REVLib 2027 needs to know which one the device is on
    public static SparkBase createMotor(MotorControllerType type, CANPort canBus, int deviceId, MotorType motorType) {
        switch (type) {
            case SPARK_MAX -> {
                return new SparkMax(canBus, deviceId, motorType);
            }
            case SPARK_FLEX -> {
                return new SparkFlex(canBus, deviceId, motorType);
            }
            default -> throw new IllegalArgumentException("Unsupported motor type: " + type);
        }
    }
}