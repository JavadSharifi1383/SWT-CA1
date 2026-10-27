
package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double SAFE_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double POSITIVE_LANE_DEVIATION_METERS = 1.0;
    private static final double NEGATIVE_LANE_DEVIATION_METERS = -1.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    private static final String SPEED_SENSOR_FAILURE_MESSAGE =
            "Speed sensor failed";
    private static final String OBSTACLE_SENSOR_FAILURE_MESSAGE =
            "Obstacle sensor failed";

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                return TARGET_SPEED_KMH;
            }
        };

        obstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                return SAFE_OBSTACLE_DISTANCE_METERS;
            }
        };

        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return 0.0;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );
    }

    @Test
    void tick_NormalConditions_NoBraking() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertFalse(result.safeMode());
        assertEquals(AdaptiveCruise.Mode.HOLD, result.cruiseMode());
        assertEquals(0.0, result.brakePressure());
        assertEquals(0.0, result.steeringAngleDegrees());
        assertFalse(result.warningActive());
    }

    @Test
    void tick_PositiveLaneDeviation_NegativeSteeringAngle() {
        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return POSITIVE_LANE_DEVIATION_METERS;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.steeringAngleDegrees() < 0.0);
    }

    @Test
    void tick_NegativeLaneDeviation_PositiveSteeringAngle() {
        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return NEGATIVE_LANE_DEVIATION_METERS;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.steeringAngleDegrees() > 0.0);
    }

    @Test
    void tick_SpeedSensorFailure_EntersSafeMode() {
        speedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                throw new SensorFailureException(
                        SPEED_SENSOR_FAILURE_MESSAGE
                );
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.safeMode());
        assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure());
        assertEquals(0.0, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }

    @Test
    void tick_ObstacleSensorFailure_EntersSafeMode() {
        obstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                throw new SensorFailureException(
                        OBSTACLE_SENSOR_FAILURE_MESSAGE
                );
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.safeMode());
        assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure());
        assertEquals(0.0, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }
}
