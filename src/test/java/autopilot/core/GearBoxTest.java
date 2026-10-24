package autopilot.core;

import autopilot.core.GearBox.Gear;
import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
class GearBoxTest {

    private static final double STOPPED_SPEED_KMH = 0.0;
    private static final double BELOW_REVERSE_LIMIT_SPEED_KMH = 4.9;
    private static final double EXACT_REVERSE_LIMIT_SPEED_KMH = 5.0;
    private static final double ABOVE_REVERSE_LIMIT_SPEED_KMH = 5.1;
    private static final double INVALID_NEGATIVE_SPEED_KMH = -30.0;
    private static final double MINIMAL_MOVING_SPEED_KMH = 0.1;
    private static final double NORMAL_MOVING_SPEED_KMH = 25.0;
    private double stubbedSpeedKmh;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        stubbedSpeedKmh = STOPPED_SPEED_KMH;
        SpeedSensor speedSensorStub = () -> stubbedSpeedKmh;
        gearBox = new GearBox(speedSensorStub);
    }

    @Test
    void changeTo_TargetGearIsNull_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class, () -> {
            gearBox.changeTo(null);
        });
    }

    @ParameterizedTest
    @ValueSource(doubles = {INVALID_NEGATIVE_SPEED_KMH, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void changeTo_SpeedIsNegativeOrNonFinite_ThrowsIllegalArgumentException(double invalidSpeed) {
        stubbedSpeedKmh = invalidSpeed;

        assertThrows(IllegalArgumentException.class, () -> {
            gearBox.changeTo(Gear.R);
        });
    }

    @ParameterizedTest
    @EnumSource(Gear.class)
    void changeTo_TargetGearEqualsCurrentGear_RemainsInSameGear(Gear targetGear) {
        transitionToGear(targetGear);
        gearBox.changeTo(targetGear);
        assertEquals(targetGear, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @ValueSource(doubles = {STOPPED_SPEED_KMH, BELOW_REVERSE_LIMIT_SPEED_KMH})
    void changeTo_FromParkToReverseAndSpeedBelowLimit_ChangesToReverse(double validSpeed) {
        stubbedSpeedKmh = validSpeed;

        gearBox.changeTo(Gear.R);

        assertEquals(Gear.R, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @ValueSource(doubles = {EXACT_REVERSE_LIMIT_SPEED_KMH, ABOVE_REVERSE_LIMIT_SPEED_KMH})
    void changeTo_FromParkToReverseAndSpeedAtOrAboveLimit_ThrowsIllegalGearChangeException(double invalidSpeed) {
        stubbedSpeedKmh = invalidSpeed;

        assertThrows(IllegalGearChangeException.class, () -> {
            gearBox.changeTo(Gear.R);
        });
    }

    @ParameterizedTest
    @MethodSource("provideAllowedTransitions")
    void changeTo_TransitionAndSpeedAreAllowed_ChangesGearSuccessfully(Gear fromGear, Gear toGear, double speed) {
        transitionToGear(fromGear);
        stubbedSpeedKmh = speed;

        gearBox.changeTo(toGear);

        assertEquals(toGear, gearBox.getCurrentGear());
    }
    private static Stream<Arguments> provideAllowedTransitions() {
        return Stream.of(
                Arguments.of(Gear.R, Gear.D, STOPPED_SPEED_KMH),
                Arguments.of(Gear.D, Gear.R, STOPPED_SPEED_KMH),
                Arguments.of(Gear.R, Gear.N, STOPPED_SPEED_KMH),
                Arguments.of(Gear.R, Gear.N, NORMAL_MOVING_SPEED_KMH),
                Arguments.of(Gear.N, Gear.D, STOPPED_SPEED_KMH),
                Arguments.of(Gear.N, Gear.D, NORMAL_MOVING_SPEED_KMH)
        );
    }

    @ParameterizedTest
    @MethodSource("provideDisallowedTransitions")
    void changeTo_TransitionOrSpeedNotAllowed_ThrowsIllegalGearChangeException(Gear fromGear, Gear toGear, double speed) {
        transitionToGear(fromGear);
        stubbedSpeedKmh = speed;

        assertThrows(IllegalGearChangeException.class, () -> {
            gearBox.changeTo(toGear);
        });
    }

    private static Stream<Arguments> provideDisallowedTransitions() {
        return Stream.of(
                Arguments.of(Gear.P, Gear.D, STOPPED_SPEED_KMH),
                Arguments.of(Gear.P, Gear.N, STOPPED_SPEED_KMH),
                Arguments.of(Gear.R, Gear.P, STOPPED_SPEED_KMH),
                Arguments.of(Gear.N, Gear.P, STOPPED_SPEED_KMH),
                Arguments.of(Gear.N, Gear.R, STOPPED_SPEED_KMH),
                Arguments.of(Gear.D, Gear.N, STOPPED_SPEED_KMH),
                Arguments.of(Gear.D, Gear.P, STOPPED_SPEED_KMH),
                Arguments.of(Gear.R, Gear.D, MINIMAL_MOVING_SPEED_KMH),
                Arguments.of(Gear.D, Gear.R, MINIMAL_MOVING_SPEED_KMH)
        );
    }

    private void transitionToGear(Gear target) {
        stubbedSpeedKmh = STOPPED_SPEED_KMH;
        if (target == Gear.P) {
            return;
        }
        gearBox.changeTo(Gear.R);
        if (target == Gear.R) {
            return;
        }
        if (target == Gear.N) {
            gearBox.changeTo(Gear.N);
        } else if (target == Gear.D) {
            gearBox.changeTo(Gear.D);
        }
    }
}