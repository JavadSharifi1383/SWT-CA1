
package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double CURRENT_SPEED_KMH = 80.0;
    private static final double SAFE_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;

    private static final double POSITIVE_LANE_DEVIATION_METERS = 1.0;
    private static final double NEGATIVE_LANE_DEVIATION_METERS = -1.0;
    private static final double RIGHT_CORRECTION_ANGLE_DEGREES = 45.0;
    private static final double LEFT_CORRECTION_ANGLE_DEGREES = -45.0;

    private static final double ZERO_STEERING_ANGLE_DEGREES = 0.0;
    private static final double ZERO_BRAKE_PRESSURE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    private static final String SPEED_SENSOR_FAILURE_MESSAGE = "Speed sensor failed";
    private static final String OBSTACLE_SENSOR_FAILURE_MESSAGE = "Obstacle sensor failed";

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = () -> CURRENT_SPEED_KMH;
        obstacleSensor = () -> SAFE_OBSTACLE_DISTANCE_METERS;
        laneSensor = () -> CENTERED_LANE_OFFSET_METERS;

        autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );
    }

    @Test
    void tick_NormalSensorsAtTargetSpeedAndCenteredLane_ReturnsHoldModeWithNoBrakingOrSteering() {
        AutoDriver.DriveResult expectedResult = new AutoDriver.DriveResult(
                ZERO_STEERING_ANGLE_DEGREES,
                ZERO_BRAKE_PRESSURE,
                AdaptiveCruise.Mode.HOLD,
                false,
                false
        );

        AutoDriver.DriveResult actualResult = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedResult, actualResult);
    }

    @ParameterizedTest
    @MethodSource("provideLaneDeviationsAndExpectedSteeringAngles")
    void tick_LaneOffsetExceedsThreshold_CorrectsSteeringAngleTowardCenter(
            double laneOffsetMeters,
            double expectedSteeringAngleDegrees
    ) {
        laneSensor = () -> laneOffsetMeters;
        autoDriver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedSteeringAngleDegrees, result.steeringAngleDegrees());
    }

    static Stream<Arguments> provideLaneDeviationsAndExpectedSteeringAngles() {
        return Stream.of(
                Arguments.of(POSITIVE_LANE_DEVIATION_METERS, LEFT_CORRECTION_ANGLE_DEGREES),
                Arguments.of(NEGATIVE_LANE_DEVIATION_METERS, RIGHT_CORRECTION_ANGLE_DEGREES)
        );
    }

    @ParameterizedTest
    @MethodSource("provideFailingSensorConfigurations")
    void tick_WhenAnySensorFails_EntersSafeModeWithDefaultBrakePressure(
            SpeedSensor configuredSpeedSensor,
            ObstacleSensor configuredObstacleSensor,
            LaneSensor configuredLaneSensor
    ) {
        autoDriver = new AutoDriver(
                configuredSpeedSensor,
                configuredObstacleSensor,
                configuredLaneSensor,
                TARGET_SPEED_KMH
        );
        AutoDriver.DriveResult expectedSafeModeResult = new AutoDriver.DriveResult(
                ZERO_STEERING_ANGLE_DEGREES,
                SAFE_MODE_BRAKE_PRESSURE,
                AdaptiveCruise.Mode.BRAKE,
                false,
                true
        );

        AutoDriver.DriveResult actualResult = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedSafeModeResult, actualResult);
    }

    static Stream<Arguments> provideFailingSensorConfigurations() {
        SpeedSensor healthySpeedSensor = () -> CURRENT_SPEED_KMH;
        ObstacleSensor healthyObstacleSensor = () -> SAFE_OBSTACLE_DISTANCE_METERS;
        LaneSensor healthyLaneSensor = () -> CENTERED_LANE_OFFSET_METERS;

        SpeedSensor failingSpeedSensor = () -> {
            throw new SensorFailureException(SPEED_SENSOR_FAILURE_MESSAGE);
        };
        ObstacleSensor failingObstacleSensor = () -> {
            throw new SensorFailureException(OBSTACLE_SENSOR_FAILURE_MESSAGE);
        };

        return Stream.of(
                Arguments.of(failingSpeedSensor, healthyObstacleSensor, healthyLaneSensor),
                Arguments.of(healthySpeedSensor, failingObstacleSensor, healthyLaneSensor)
        );
    }
}