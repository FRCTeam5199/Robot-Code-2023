package frc.drive.auton.pointtopoint;


import frc.drive.auton.Point;

import static frc.drive.auton.pointtopoint.AutonSpecialActions.NONE;

public class AutonWaypoint {
    public final Point LOCATION;
    public final double SPEED;
    public final AutonSpecialActions SPECIAL_ACTION;
    public final int INTARG, INTARG2, INTARG3;
    public final AutonSpecialActions SPECIAL_ACTION_2;


    public AutonWaypoint(Point pos, double speed, AutonSpecialActions specialAction, int intarg, AutonSpecialActions specialActions2, int intarg2, int intarg3) {
        LOCATION = pos;
        SPEED = speed;
        SPECIAL_ACTION = specialAction;
        INTARG = intarg;
        INTARG2 = intarg2;
        INTARG3 = intarg3;
        SPECIAL_ACTION_2 = specialActions2;
    }


    public AutonWaypoint(Point pos, double speed, AutonSpecialActions specialAction) {
        this(pos, speed, specialAction, 0, NONE, 0, 0);
    }

    public AutonWaypoint(Point pos, double speed, AutonSpecialActions specialAction, int intarg) {
        this(pos, speed, specialAction, intarg, NONE, 0, 0);
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
                intarg,
                 NONE,
                0,
                0
        );
    }

    public AutonWaypoint(AutonSpecialActions action) {
        this(
                new Point(-9999, -9999),
                1,
                action,
                0,
                 NONE,
                0,
                0
        );
    }

    public AutonWaypoint(Point point, double i, AutonSpecialActions driveTo, int i1, AutonSpecialActions specialActions) {
        this(point,i,driveTo,i1,specialActions,0,0);
    }
}