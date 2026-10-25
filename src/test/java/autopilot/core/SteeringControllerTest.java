package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class SteeringControllerTest {

    private static final double ZERO_SPEED_KMH = 0.0;
    private static final double BELOW_ZERO_SPEED_KMH = -0.1;
    private static final double NORMAL_SPEED_KMH = 60.0;
    private static final double HIGH_SPEED_THRESHOLD_KMH = 120.0;
    private static final double ABOVE_HIGH_SPEED_THRESHOLD_KMH = 120.1;

    private static final double VALID_MODERATE_ANGLE_DEGREES = 35.0;
    private static final double PHYSICAL_LIMIT_DEGREES = 450.0;
    private static final double BEYOND_PHYSICAL_LIMIT_DEGREES = 500.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double BEYOND_HIGH_SPEED_LIMIT_DEGREES = 110.0;

    private static final String INVALID_ANGLE_MESSAGE = "requested angle must be finite";
    private static final String INVALID_SPEED_MESSAGE = "speed must be a finite non-negative value";

    private SteeringController steeringController;

    @BeforeEach
    void setUp() {
        steeringController = new SteeringController();
    }

    @ParameterizedTest
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_AngleIsNonFinite_ThrowsIllegalArgumentException(double nonFiniteAngle) {
        IllegalArgumentException exception = assertThrows(IllegalArgumentException.class, () ->
                steeringController.update(nonFiniteAngle, NORMAL_SPEED_KMH)
        );

        assertEquals(INVALID_ANGLE_MESSAGE, exception.getMessage());
    }

    @ParameterizedTest
    @ValueSource(doubles = {BELOW_ZERO_SPEED_KMH, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_SpeedIsNegativeOrNonFinite_ThrowsIllegalArgumentException(double invalidSpeed) {
        IllegalArgumentException exception = assertThrows(IllegalArgumentException.class, () ->
                steeringController.update(VALID_MODERATE_ANGLE_DEGREES, invalidSpeed)
        );

        assertEquals(INVALID_SPEED_MESSAGE, exception.getMessage());
    }

    @ParameterizedTest
    @MethodSource("provideAnglesAndSpeedsForClamping")
    void update_RequestedAngleAndSpeed_ClampsTargetAngleCorrectly(
            double requestedAngle,
            double speedKmh,
            double expectedClampedTarget
    ) {
        steeringController.update(requestedAngle, speedKmh);

        assertEquals(expectedClampedTarget, steeringController.getTargetAngleDegrees());
    }

    static Stream<Arguments> provideAnglesAndSpeedsForClamping() {
        return Stream.of(
                Arguments.of(VALID_MODERATE_ANGLE_DEGREES, ZERO_SPEED_KMH, VALID_MODERATE_ANGLE_DEGREES),
                Arguments.of(PHYSICAL_LIMIT_DEGREES, NORMAL_SPEED_KMH, PHYSICAL_LIMIT_DEGREES),
                Arguments.of(BEYOND_PHYSICAL_LIMIT_DEGREES, NORMAL_SPEED_KMH, PHYSICAL_LIMIT_DEGREES),
                Arguments.of(-PHYSICAL_LIMIT_DEGREES, NORMAL_SPEED_KMH, -PHYSICAL_LIMIT_DEGREES),
                Arguments.of(-BEYOND_PHYSICAL_LIMIT_DEGREES, NORMAL_SPEED_KMH, -PHYSICAL_LIMIT_DEGREES),
                Arguments.of(BEYOND_PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH, PHYSICAL_LIMIT_DEGREES),
                Arguments.of(HIGH_SPEED_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH, HIGH_SPEED_LIMIT_DEGREES),
                Arguments.of(BEYOND_HIGH_SPEED_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH, HIGH_SPEED_LIMIT_DEGREES),
                Arguments.of(-HIGH_SPEED_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH, -HIGH_SPEED_LIMIT_DEGREES),
                Arguments.of(-BEYOND_HIGH_SPEED_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH, -HIGH_SPEED_LIMIT_DEGREES)
        );
    }
    @ParameterizedTest
    @CsvFileSource(resources = "/steering-movements.csv", numLinesToSkip = 1)
    void update_FromInitialAngle_MovesTowardTargetWithinMaxChangePerTick(
            double initialCurrentAngle,
            double requestedAngle,
            double expectedCurrentAngle
    ) {
        reachInitialAngle(initialCurrentAngle);

        double actualCurrentAngle = steeringController.update(requestedAngle, NORMAL_SPEED_KMH);

        assertEquals(expectedCurrentAngle, actualCurrentAngle);
    }

    @ParameterizedTest
    @CsvSource({
            "30.0, 0.0",
            "-30.0, 0.0",
            "100.0, 40.0",
            "-100.0, -40.0"
    })
    void selfCenter_FromNonZeroAngle_MovesTowardZeroWithinMaxChangePerTick(
            double initialCurrentAngle,
            double expectedAngleAfterOneSelfCenter
    ) {
        reachInitialAngle(initialCurrentAngle);

        double actualAngle = steeringController.selfCenter();

        assertEquals(expectedAngleAfterOneSelfCenter, actualAngle);
    }

    @ParameterizedTest
    @CsvSource({
            "50.0, 200.0, 130.0, 90.0",
            "130.0, 130.0, 140.0, 90.0",
            "200.0, 200.0, 140.0, 140.0",
            "410.0, 500.0, 60.0, 450.0",
            "40.0, -200.0, 130.0, -20.0"
    })
    void update_CombinedSpeedClampingAndRateLimiting_UpdatesCurrentAngleCorrectly(
            double initialCurrentAngle,
            double requestedAngle,
            double speedKmh,
            double expectedCurrentAngle
    ) {
        reachInitialAngle(initialCurrentAngle);

        double actualCurrentAngle = steeringController.update(requestedAngle, speedKmh);

        assertEquals(expectedCurrentAngle, actualCurrentAngle);
    }

    private void reachInitialAngle(double targetInitialAngle) {
        while (steeringController.getCurrentAngleDegrees() != targetInitialAngle) {
            steeringController.update(targetInitialAngle, NORMAL_SPEED_KMH);
        }
    }
}