package frc.robot.constants;

import static org.wpilib.units.Units.*;

import org.wpilib.units.measure.Distance;
import org.wpilib.driverstation.Alliance;
import java.util.Optional;

public final class FieldConstants {

    public static record Pose(String name) {}

    public static Optional<Alliance> alliance = Optional.empty();

    // values taken from field drawings
    public static final Distance HALF_WIDTH_OF_HUB = Inches.of(47.0 / 2.0);
    public static final Distance HALF_LENGTH_OF_ROBOT_WITH_BUMPER = Inches.of(15);
    public static final Distance ROBOT_HUB_MARGIN = Inches.of(15);
    public static final Distance HALF_WIDTH_OF_ROBOT_WITH_BUMPER = Inches.of(15);
    public static final Distance STARTING_LINE_TO_CENTER_DISTANCE = Inches.of(47 + 120);
    public static final Distance FUEL_PIT_HALF_WIDTH = Inches.of(35.95);
    public static final Distance LENGTH_OF_BUMP = Inches.of(73.00 + 12.00);

    public static class FieldBoundaries {
        public static final Distance RED_ZONE_BOUNDARY_X = Inches.of(181.56 + (2 * 143.50));
        public static final Distance RED_DRIVER_STATION_WALL_X = Meters.of(16.94);
        public static final Distance BLUE_ZONE_BOUNDARY_X = Inches.of(181.56);
        public static final Distance BLUE_DRIVER_STATION_WALL_X = Meters.of(0);
        public static final Distance SCREEN_LINE =
                Inches.of(204); // screen 17 ft from scoring table (measured by fta)
        public static final Distance HORIZONTAL_CENTER_LINE = Meters.of(4.034663);
        public static final Distance SCREEN_RANGE = Meters.of(0);
    }

    public static class HubConstants {
        public static class AlignmentTargetPoseConstants {
            public static final Distance HUB_RADIUS = Inches.of(23.5);
            public static final Distance FUEL_RADIUS = Inches.of(2.955);

            // values taken from field drawings
            
        }
    }
}
