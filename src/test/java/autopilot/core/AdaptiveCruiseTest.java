package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class AdaptiveCruiseTest {
    private static final double TARGET_SPEED = 100.0;
    private static final double CURRENT_SPEED = 60.0;
    private static final double SAFE_DISTANCE = 30.0;
    private static final double TOLERANCE = 0.01;

    private AdaptiveCruise cruise;

    @BeforeEach
    void setUp() {
        cruise = new AdaptiveCruise(TARGET_SPEED);
    }

    @ParameterizedTest
    @CsvSource({
            "0.0, 0.0",
            "60.0, 30.0",
            "100.0, 50.0"
    })
    void safeDistanceFor_ValidSpeed_ReturnsCorrectDistance(double speedKmh, double expectedDistance) {
        assertEquals(expectedDistance, cruise.safeDistanceFor(speedKmh), TOLERANCE);
    }

    @ParameterizedTest
    @CsvSource({
            "0.0, 100.0",
            "14.0, 100.0",
            "15.0, 100.0"
    })
    void update_DistanceAtOrBelowEmergencyThreshold_ReturnsEmergencyBrake(double distanceMeters, double expectedAmount) {
        AdaptiveCruise.Decision decision = cruise.update(CURRENT_SPEED, distanceMeters);

        assertEquals(AdaptiveCruise.Mode.EMERGENCY_BRAKE, decision.mode());
        assertEquals(expectedAmount, decision.amount(), TOLERANCE);
    }

    @ParameterizedTest
    @CsvSource({
            "16.0, 46.6666666667",
            "20.0, 33.3333333333"
    })
    void update_DistanceBetweenEmergencyAndSafeThreshold_ReturnsBrake(double distanceMeters, double expectedAmount) {
        AdaptiveCruise.Decision decision = cruise.update(CURRENT_SPEED, distanceMeters);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(expectedAmount, decision.amount(), TOLERANCE);
    }

    @ParameterizedTest
    @ValueSource(doubles = {30.0, 33.0})
    void update_BrakingAndDistanceWithinReleaseThreshold_ReturnsBrake(double distanceMeters) {
        cruise.update(CURRENT_SPEED, 10.0);

        AdaptiveCruise.Decision decision = cruise.update(CURRENT_SPEED, distanceMeters);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(10.0, decision.amount(), TOLERANCE);
    }

    @Test
    void update_BrakingAndDistanceEqualsReleaseThreshold_ReleasesBrake() {
        cruise.update(CURRENT_SPEED, 10.0);

        AdaptiveCruise.Decision decision = cruise.update(CURRENT_SPEED, 36.0);

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(5.0, decision.amount(), TOLERANCE);
    }

    @ParameterizedTest
    @CsvSource({
            "100.0, 50.0",
            "100.0, 50.1",
            "120.0, 65.0"
    })
    void update_DistanceAtOrAboveSafeDistanceAndSpeedAtOrAboveTarget_ReturnsHold(double speedKmh, double distanceMeters) {
        AdaptiveCruise.Decision decision = cruise.update(speedKmh, distanceMeters);

        assertEquals(AdaptiveCruise.Mode.HOLD, decision.mode());
        assertEquals(0.0, decision.amount(), TOLERANCE);
    }

    @Test
    void update_DistanceAboveSafeDistanceAndBelowTargetSpeed_ReturnsAccelerate() {
        AdaptiveCruise.Decision decision = cruise.update(CURRENT_SPEED, 33.0);

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(5.0, decision.amount(), TOLERANCE);
    }

    @Test
    void update_SpeedSlightlyBelowTargetSpeed_AcceleratesOnlyToTarget() {
        AdaptiveCruise.Decision decision = cruise.update(98.0, 60.0);

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(2.0, decision.amount(), TOLERANCE);
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidSpeed_ThrowsIllegalArgumentException(double speedKmh) {
        assertThrows(IllegalArgumentException.class, () -> cruise.update(speedKmh, SAFE_DISTANCE));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidDistance_ThrowsIllegalArgumentException(double distanceMeters) {
        assertThrows(IllegalArgumentException.class, () -> cruise.update(CURRENT_SPEED, distanceMeters));
    }
}