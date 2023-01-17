package frc.drive.auton.pointtopoint;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.drive.auton.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import static frc.drive.auton.pointtopoint.AutonSpecialActions.*;

/**
 * All the auton paths, using {@link Point points} and {@link AutonSpecialActions} to make the robot do the do
 */
public enum AutonRoutines {
    DO_NOTHING(
            new AutonWaypoint(new Point(0, 0))
    ),
    DRIVE_OFF_INIT_LINE(
            new AutonWaypoint(new Point(0, 0), 1, DRIVE_TO, 1),
            new AutonWaypoint(new Point(8, -2), 1, DRIVE_TO, 90)
    ),
    PID_AND_APRIL_TEST(
            new AutonWaypoint(new Point(-6, 2), .5, DRIVE_TO, 1),
            new AutonWaypoint(new Point(-13, 2), .5, DRIVE_TO, 1)
    ),
    SPIN_TEST(
            new AutonWaypoint(new Point(0, 0), 1, DRIVE_TO, 90)
    ),
    NEO_DRIVE(
            new AutonWaypoint(new Point(0,0)),
            new AutonWaypoint(new Point(1.25, 0), DRIVE_TO),
            new AutonWaypoint(new Point(1.5, 0), 1, DRIVE_TO, 1),
            new AutonWaypoint(new Point(2.5, 0), 1, ROTATE_ROBOT, 180)
    );

    private static SendableChooser<AutonRoutines> myChooser;
    public final ArrayList<AutonWaypoint> WAYPOINTS = new ArrayList<>();
    public int currentWaypoint = 0;

    public static SendableChooser<AutonRoutines> getSendableChooser() {
        return Objects.requireNonNullElseGet(myChooser, () -> {
            myChooser = new SendableChooser<>();
            for (AutonRoutines routine : AutonRoutines.values())
                myChooser.addOption(routine.name(), routine);
            return myChooser;
        });
    }

    AutonRoutines(AutonWaypoint... waypoints) {
        WAYPOINTS.addAll(Arrays.asList(waypoints));
    }
}