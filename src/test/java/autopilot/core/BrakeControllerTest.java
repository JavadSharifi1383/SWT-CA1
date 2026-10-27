package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BrakeControllerTest {

    private static final double EXPECTED_MAX_PRESSURE = 100.0;
    private static final double EXPECTED_ABS_HIGH_PULSE = 90.0;
    private static final double EXPECTED_ABS_LOW_PULSE = 60.0;
    private static final double NORMAL_SPEED = 20.0;
    private static final double ABS_SPEED = 40.0;
    private static final double NORMAL_PRESSURE = 50.0;
    private static final double HIGH_PRESSURE = 150.0;
    private static final double ABS_PRESSURE = 90.0;

    private BrakeController brakeController;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    @ParameterizedTest
    @CsvSource({
            "0.0, 0.0",
            "50.0, 20.0",
            "80.0, 30.0",
            "90.0, 30.0",
            "79.0, 50.0"
    })
    void apply_AbsConditionsNotMet_ReturnsRequestedPressure(double requestedPressure, double speedKmh) {
        double resultPressure = brakeController.apply(requestedPressure, speedKmh, GearBox.Gear.D);

        assertEquals(requestedPressure, resultPressure);
    }

    @ParameterizedTest
    @CsvSource({
            "80.1, 30.1",
            "90.0, 40.0",
            "100.0, 100.0"
    })
    void apply_AbsConditionsMet_ReturnsHighPulseOnFirstCall(double requestedPressure, double speedKmh) {
        double resultPressure = brakeController.apply(requestedPressure, speedKmh, GearBox.Gear.D);

        assertEquals(EXPECTED_ABS_HIGH_PULSE, resultPressure);
    }

    @Test
    void apply_AbsConditionsMetOnConsecutiveCalls_AlternatesPulses() {
        double firstResult = brakeController.apply(ABS_PRESSURE, ABS_SPEED, GearBox.Gear.D);
        double secondResult = brakeController.apply(ABS_PRESSURE, ABS_SPEED, GearBox.Gear.D);

        assertEquals(EXPECTED_ABS_HIGH_PULSE, firstResult);
        assertEquals(EXPECTED_ABS_LOW_PULSE, secondResult);
    }

    @ParameterizedTest
    @ValueSource(doubles = {
            -1.0,
            -10.0,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
    })
    void apply_InvalidPressure_ThrowsIllegalArgumentException(double invalidPressure) {
        assertThrows(IllegalArgumentException.class, () -> brakeController.apply(invalidPressure, NORMAL_SPEED, GearBox.Gear.D));
    }

    @ParameterizedTest
    @MethodSource("invalidSpeeds")
    void apply_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> brakeController.apply(NORMAL_PRESSURE, invalidSpeed, GearBox.Gear.D));
    }

    private static Stream<Double> invalidSpeeds() {
        return Stream.of(
                -1.0,
                -5.0,
                Double.NaN,
                Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY
        );
    }

    @Test
    void apply_BrakingInPark_ThrowsIllegalStateException() {
        assertThrows(IllegalStateException.class, () -> brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED, GearBox.Gear.P));
    }

    @ParameterizedTest
    @CsvFileSource(resources = "/brake-pressure-cases.csv", numLinesToSkip = 1)
    void apply_ValidPressureAndSpeed_ReturnsExpectedPressure(double requestedPressure, double speedKmh, double expectedPressure) {
        double resultPressure = brakeController.apply(requestedPressure, speedKmh, GearBox.Gear.D);

        assertEquals(expectedPressure, resultPressure);
    }

    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, names = {"P"}, mode = EnumSource.Mode.EXCLUDE)
    void apply_BrakingInAllowedGear_ReturnsRequestedPressure(GearBox.Gear gear) {
        double resultPressure = brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED, gear);

        assertEquals(NORMAL_PRESSURE, resultPressure);
    }

    @Test
    void apply_PressureAboveMaximum_ReturnsMaximumPressure() {
        double resultPressure = brakeController.apply(HIGH_PRESSURE, NORMAL_SPEED, GearBox.Gear.D);

        assertEquals(EXPECTED_MAX_PRESSURE, resultPressure);
    }

    @Test
    void emergencyBrake_AllowedGear_AppliesMaximumPressureAndActivatesWarning() {
        brakeController.emergencyBrake(GearBox.Gear.D);

        assertEquals(EXPECTED_MAX_PRESSURE, brakeController.getPressure());
        assertTrue(brakeController.isWarningActive());
    }

    @Test
    void emergencyBrake_AllowedGear_ResetsNextAbsPulseToHigh() {
        brakeController.apply(ABS_PRESSURE, ABS_SPEED, GearBox.Gear.D);
        brakeController.emergencyBrake(GearBox.Gear.D);

        double resultPressure = brakeController.apply(ABS_PRESSURE, ABS_SPEED, GearBox.Gear.D);

        assertEquals(EXPECTED_ABS_HIGH_PULSE, resultPressure);
    }

    @Test
    void emergencyBrake_BrakingInPark_ThrowsIllegalStateException() {
        assertThrows(IllegalStateException.class, () -> brakeController.emergencyBrake(GearBox.Gear.P));
    }

    @Test
    void apply_NullGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class, () -> brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED, null));
    }

    @Test
    void emergencyBrake_NullGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class, () -> brakeController.emergencyBrake(null));
    }
}