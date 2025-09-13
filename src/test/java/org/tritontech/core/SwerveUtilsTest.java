package org.tritontech.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveUtilsTest {
    private static final double DELTA = 1e-10; // Tolerance for floating-point comparisons
    private static final double TWO_PI = 2 * Math.PI;

    @BeforeEach
    void setUp() {
        // No instance setup needed since methods are static
    }

    // Tests for AngleDifference
    @Test
    void testAngleDifferenceWithinRange() {
        // Test angles within [0, 2π) where difference is straightforward
        double angleA = Math.PI / 2; // 90 degrees
        double angleB = Math.PI; // 180 degrees
        double expected = Math.PI / 2; // 90 degrees
        assertEquals(expected, SwerveUtils.AngleDifference(angleA, angleB), DELTA,
                "Angle difference within [0, 2π) should be π/2");
    }

    @Test
    void testAngleDifferenceAcrossZero() {
        // Test angles where difference crosses 0/2π boundary
        double angleA = 0.1; // Just above 0
        double angleB = TWO_PI - 0.1; // Just below 2π
        double expected = 0.2; // Shortest path is 0.2 radians
        assertEquals(expected, SwerveUtils.AngleDifference(angleA, angleB), DELTA,
                "Angle difference across 0/2π should be 0.2 radians");
    }

    @Test
    void testAngleDifferenceNegativeAngles() {
        // Test with negative angles
        double angleA = -Math.PI / 4; // -45 degrees
        double angleB = Math.PI / 4; // 45 degrees
        double expected = Math.PI / 2; // 90 degrees
        assertEquals(expected, SwerveUtils.AngleDifference(angleA, angleB), DELTA,
                "Angle difference with negative angle should be π/2");
    }

    @Test
    void testAngleDifferenceEqualAngles() {
        // Test when angles are equal
        double angleA = Math.PI;
        double angleB = Math.PI;
        double expected = 0.0;
        assertEquals(expected, SwerveUtils.AngleDifference(angleA, angleB), DELTA,
                "Angle difference for equal angles should be 0");
    }

    @Test
    void testAngleDifferenceExactlyPi() {
        // Test when difference is exactly π
        double angleA = 0.0;
        double angleB = Math.PI;
        double expected = Math.PI;
        assertEquals(expected, SwerveUtils.AngleDifference(angleA, angleB), DELTA,
                "Angle difference of exactly π should return π");
    }

    // Tests for WrapAngle
    @Test
    void testWrapAngleWithinRange() {
        // Test angle already in [0, 2π)
        double angle = Math.PI / 2; // 90 degrees
        double expected = Math.PI / 2;
        assertEquals(expected, SwerveUtils.WrapAngle(angle), DELTA,
                "Angle within [0, 2π) should remain unchanged");
    }

    @Test
    void testWrapAngleExactlyTwoPi() {
        // Test angle exactly at 2π
        double angle = TWO_PI;
        double expected = 0.0;
        assertEquals(expected, SwerveUtils.WrapAngle(angle), DELTA,
                "Angle of exactly 2π should wrap to 0");
    }

    @Test
    void testWrapAnglePositiveMultipleWraps() {
        // Test angle requiring multiple wraps (> 2π)
        double angle = TWO_PI + Math.PI; // 2π + π
        double expected = Math.PI;
        assertEquals(expected, SwerveUtils.WrapAngle(angle), DELTA,
                "Angle > 2π should wrap to π");
    }

    @Test
    void testWrapAngleNegative() {
        // Test negative angle requiring wrapping
        double angle = -Math.PI / 2; // -90 degrees
        double expected = TWO_PI - Math.PI / 2; // 270 degrees
        assertEquals(expected, SwerveUtils.WrapAngle(angle), DELTA,
                "Negative angle should wrap to 3π/2");
    }

    @Test
    void testWrapAngleNegativeMultipleWraps() {
        // Test negative angle requiring multiple wraps
        double angle = -TWO_PI - Math.PI; // -2π - π
        double expected = Math.PI;
        assertEquals(expected, SwerveUtils.WrapAngle(angle), DELTA,
                "Negative angle with multiple wraps should wrap to π");
    }

    @Test
    void testWrapAngleZero() {
        // Test angle of 0
        double angle = 0.0;
        double expected = 0.0;
        assertEquals(expected, SwerveUtils.WrapAngle(angle), DELTA,
                "Angle of 0 should remain 0");
    }
}