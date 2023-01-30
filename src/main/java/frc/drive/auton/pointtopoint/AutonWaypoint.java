package frc.drive.auton.pointtopoint;


import frc.drive.auton.Point;

import static frc.drive.auton.pointtopoint.AutonSpecialActions.NONE;

public class AutonWaypoint {
    public final Point LOCATION;
    public final double SPEED;
    public final AutonSpecialActions SPECIAL_ACTION;
    //public AutonSpecialActions SPECIAL_ACTION_2 = NONE;
    public final int INTARG;

    public AutonWaypoint(Point pos, double speed, AutonSpecialActions specialAction, int intarg) {
        LOCATION = pos;
        SPEED = speed;
        SPECIAL_ACTION = specialAction;
        INTARG = intarg;
    }
    /*
    public AutonWaypoint(Point pos, double speed, AutonSpecialActions specialAction, int intarg, AutonSpecialActions specialActions2) {
        LOCATION = pos;
        SPEED = speed;
        SPECIAL_ACTION = specialAction;
        INTARG = intarg;
        SPECIAL_ACTION_2 = specialActions2;
    }
    */
    public AutonWaypoint(Point pos, double speed, AutonSpecialActions specialAction) {
        this(pos, speed, specialAction, 0);
    }

    public AutonWaypoint(double x, double y, double speed) {
        this(new Point(x, y), speed);
    }

    //----------------------------------------------------------------------------------------
//Anything below here has a default of no action.
//----------------------------------------------------------------------------------------
    public AutonWaypoint(Point pos, double speed) {
        this(pos, speed, NONE);
    }

    public AutonWaypoint(Point pos, AutonSpecialActions action) {
        this(pos, 1, action);
    }

    public AutonWaypoint(Point pos) {
        this(pos, 1);
    }

    public AutonWaypoint(AutonSpecialActions action, int intarg) {
        this(
                new Point(-9999, -9999),
                1,
                action,
                intarg
        );
    }

    public AutonWaypoint(AutonSpecialActions action) {
        this(
                new Point(-9999, -9999),
                1,
                action,
                0
        );
    }
}