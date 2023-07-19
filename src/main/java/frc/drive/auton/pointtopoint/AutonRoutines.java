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
    DO_NOTHING_RED(
            new AutonWaypoint(new Point(3.2, 0) ,.25,DRIVE_TO, 1)
    ),
    DO_NOTHING_BLUE(
            new AutonWaypoint(new Point(-54 + 3.2, 0) ,.25,DRIVE_TO, 180)
    ),
    AUTON_PLACE_2_ANTIHP_RED(
            new AutonWaypoint(new Point(5.4, 2), .25, DRIVE_TO, 2),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT500),
            new AutonWaypoint(new Point(5.4, 2), 1, DRIVE_TO, 2, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(5.4, 2), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(5.4, 2), 1, DRIVE_TO, 2,    ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(5.4, 2), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, -2,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_IN),
            //new AutonWaypoint(new Point(-17, 14.2), .5, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-33),,
            new AutonWaypoint(new Point(18, 5.5), .37, DRIVE_TO, 0, ARM_ELEVATOR_GO_TO, -44,-140),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_OUT),
            new AutonWaypoint(new Point(25.5, 5.7), .34, DRIVE_TO, 1),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_IN),
            new AutonWaypoint(new Point(8.6, 4), .3, DRIVE_TO, 1),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OFF),
            new AutonWaypoint(new Point(8.6, 4), .3, DRIVE_TO, 170),
            //new AutonWaypoint(new Point(-10, 15), .37, DRIVE_TO, -179),
            //new AutonWaypoint(new Point(-8, 14.5), .37, DRIVE_TO, -179),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OUT),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    AUTON_PLACE_2_ANTIHP_BLUE(
            new AutonWaypoint(new Point(-54 + 5.4, 1.1), .25, DRIVE_TO, 179),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT500),
            new AutonWaypoint(new Point(-54 + 5.4, 1.1), 1, DRIVE_TO, 179, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-54 + 5.4, 1.1), 1, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-54 + 5.4, 1.1), 1, DRIVE_TO, 179,    ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-54 + 5.4, 1.1), 1, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, -2,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_IN),
            //new AutonWaypoint(new Point(-17, 14.2), .5, DR
            // IVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-33),,
            new AutonWaypoint(new Point(-54 + 18, 1.3), .37, DRIVE_TO, 180, ARM_ELEVATOR_GO_TO, -44,-140),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_OUT),
            new AutonWaypoint(new Point(-54 + 26, 1.1), .34, DRIVE_TO, 180),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_IN),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OFF),
            new AutonWaypoint(new Point(-54 + 8.8, 1.4), .3, DRIVE_TO, 180),
            new AutonWaypoint(new Point(-54 + 8.9, 2), .3, DRIVE_TO, -10),
            //new AutonWaypoint(new Point(-10, 15), .37, DRIVE_TO, -179),
            //new AutonWaypoint(new Point(-8, 14.5), .37, DRIVE_TO, -179),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OUT),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    /*
    AUTON_PLACE_2_HP_RED(
            new AutonWaypoint(new Point(-3.2, 15.2), .25, INTAKE_PISTON_IN, 2),
            new AutonWaypoint(WAIT100),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 2, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 2,   ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-3.1, 15.2), .25, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, -1,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            //new AutonWaypoint(new Point(-17, 14.2), .5, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-33),,
            new AutonWaypoint(new Point(-16, 15.4), .45, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-22, 14.9), .37, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(new Point(-5.8, 14.3), .43, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -1,-220),
            new AutonWaypoint(INTAKE_WHEEL_IN),
            new AutonWaypoint(new Point(-16, 8), 1.2, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -44,-140),
            new AutonWaypoint(AUTO_LEVEL),
            new AutonWaypoint(ARM_ELEVATOR_SHIFT_WEIGHT),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    */
    AUTON_PLACE_2_HP_RED(
            new AutonWaypoint(new Point(-5.4, 15.2), .25, DRIVE_TO, 2),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT500),
            new AutonWaypoint(new Point(-5.4, 15.2), 1, DRIVE_TO, 2, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-5.4, 15.2), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-5.4, 15.2), 1, DRIVE_TO, 2,    ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-5.4, 15.2), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, -2,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_IN),
            //new AutonWaypoint(new Point(-17, 14.2), .5, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-33),,
            new AutonWaypoint(new Point(-18, 16.1), .37, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -44,-140),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_OUT),
            new AutonWaypoint(new Point(-24.7, 15.8), .34, DRIVE_TO, 1),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_IN),
            new AutonWaypoint(new Point(-8.6, 14.9), .3, DRIVE_TO, 1),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OFF),
            new AutonWaypoint(new Point(-8.6, 12.2), .3, DRIVE_TO, 170),
            //new AutonWaypoint(new Point(-10, 15), .37, DRIVE_TO, -179),
            //new AutonWaypoint(new Point(-8, 14.5), .37, DRIVE_TO, -179),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OUT),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    AUTON_PLACE_2_HP_BLUE(
            new AutonWaypoint(new Point(-54 + 5.4, 15.2), .25, DRIVE_TO, 179),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT500),
            new AutonWaypoint(new Point(-54 + 5.4, 15.2), 1, DRIVE_TO, 179, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-54 + 5.4, 15.2), 1, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-54 + 5.4, 15.2), 1, DRIVE_TO, 179,    ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-54 + 5.4, 15.2), 1, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, -2,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_IN),
            //new AutonWaypoint(new Point(-17, 14.2), .5, DR
            // IVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-33),,
            new AutonWaypoint(new Point(-54 + 18, 11.5), .37, DRIVE_TO, 180, ARM_ELEVATOR_GO_TO, -44,-140),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_OUT),
            new AutonWaypoint(new Point(-54 + 25.5, 11.5), .34, DRIVE_TO, 180),
            new AutonWaypoint(INTAKE_PISTON_BOTTOM_IN),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OFF),
            new AutonWaypoint(new Point(-54 + 8.8, 13.4), .3, DRIVE_TO, 180),
            new AutonWaypoint(new Point(-54 + 8.8, 14), .3, DRIVE_TO, -10),
            //new AutonWaypoint(new Point(-10, 15), .37, DRIVE_TO, -179),
            //new AutonWaypoint(new Point(-8, 14.5), .37, DRIVE_TO, -179),
            new AutonWaypoint(INTAKE_WHEEL_BOTTOM_OUT),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    CLIMB_RED(
            new AutonWaypoint(new Point(-5.4, 8), .25, DRIVE_TO, 2),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT500),
            new AutonWaypoint(new Point(-5.4, 8), 1, DRIVE_TO, 2, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-5.4, 8), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-5.4, 8), 1, DRIVE_TO, 2,    ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-5.4, 8), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, -2,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(new Point(-8, 10), .6, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -44,-140),
            new AutonWaypoint(new Point(-16, 11), 1.3, DRIVE_TO, 1),
            new AutonWaypoint(AUTO_LEVEL),
            new AutonWaypoint(WAIT1000),
            new AutonWaypoint(AUTO_LEVEL),
            new AutonWaypoint(ARM_ELEVATOR_SHIFT_WEIGHT),
            new AutonWaypoint(LOCK_WHEELS)
    ),
    CLIMB_RED_MOBILITY(
            new AutonWaypoint(new Point(-5.4, 8), .25, DRIVE_TO, 2),
        //     new AutonWaypoint(INTAKE_PISTON_IN),
             new AutonWaypoint(WAIT500),
        //     new AutonWaypoint(new Point(5.4, 8), 1, DRIVE_TO, 2, ARM_ELEVATOR_UP),
        //     new AutonWaypoint(new Point(5.4, 8), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, 2,-60),
        //     new AutonWaypoint(new Point(5.4, 8), 1, DRIVE_TO, 2,    ARM_ELEVATOR_GO_TO, -3,-207),
        //     new AutonWaypoint(new Point(5.4, 8), 1, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, -2,-220),
        //     new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(DRIVE_OVER_STATION),
            new AutonWaypoint(AUTO_LEVEL)
    ),
    AUTON_PLACE_1_HP_RED_LEVEL(
            new AutonWaypoint(new Point(-5.4, 15.2), .25,DRIVE_TO),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT100),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 1, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 1,   ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-3.1, 15.2), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -2,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            new AutonWaypoint(new Point(-18, 15.5), .37, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-22, 14.9), .37, DRIVE_TO, 1, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(new Point(-20, 10), .4, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-54),
            new AutonWaypoint(new Point(-12, 10), 1.2, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-54),
            new AutonWaypoint(AUTO_LEVEL),
            new AutonWaypoint(ARM_ELEVATOR_SHIFT_WEIGHT),
            new AutonWaypoint(LOCK_WHEELS)
    ),
    CLIMB_BLUE(
            new AutonWaypoint(new Point(-54 + 5.4, 8), .25, DRIVE_TO, 179),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT500),
            new AutonWaypoint(new Point(-54 + 5.4, 8), 1, DRIVE_TO, 179, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-54 + 5.4, 8), 1, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-54 + 5.4, 8), 1, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-54 + 5.4, 8), 1, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, -2,-225),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(new Point(-54 + 5.4, 8), .5, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, -44,-140),
            new AutonWaypoint(new Point(-54 + 8, 10), .6, DRIVE_TO, 180),
            new AutonWaypoint(new Point(-54 + 16, 10), 1.3, DRIVE_TO, 180),
            new AutonWaypoint(AUTO_LEVEL),
            new AutonWaypoint(WAIT1000),
            new AutonWaypoint(AUTO_LEVEL),
            new AutonWaypoint(ARM_ELEVATOR_SHIFT_WEIGHT),
            new AutonWaypoint(LOCK_WHEELS)
    ),
    AUTON_PLACE_1_HP_BLUE_LEVEL(
            new AutonWaypoint(new Point(-54 + 3.2, 15.2), .25,DRIVE_TO),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT100),
            new AutonWaypoint(new Point(-54 + 3.2, 15.2), .25, DRIVE_TO, 1, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-54 + 3.2, 15.2), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-54 + 3.2, 15.2), .25, DRIVE_TO, 1,   ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-54 + 3.1, 15.2), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -2,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            new AutonWaypoint(new Point(-54 + 18, 15.5), .37, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-54 + 22, 14.9), .37, DRIVE_TO, 1, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(new Point(-54 + 20, 10), .4, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-54),
            new AutonWaypoint(new Point(-54 + 12, 10), 1.2, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-54),
            new AutonWaypoint(AUTO_LEVEL),
            new AutonWaypoint(ARM_ELEVATOR_SHIFT_WEIGHT),
            new AutonWaypoint(LOCK_WHEELS)
    ),
    AUTON_PLACE_2_HP_RED_NO_LEVEL(
            new AutonWaypoint(new Point(-3.2, 15.2), .25, INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT100),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 1, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 1,   ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-3.1, 15.2), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -1,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            new AutonWaypoint(new Point(-18, 15.4), .4, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-22, 14.9), .3, DRIVE_TO, 1, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(new Point(-4.4, 14.2), .4, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -1,-220),
            new AutonWaypoint(INTAKE_WHEEL_IN),
            new AutonWaypoint(LOCK_WHEELS)
    ),
    AUTON_PLACE_1_HP_RED_NO_LEVEL(
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO,2),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT100),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 2, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 2,   ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-3.2, 15.2), .25, DRIVE_TO, 2,  ARM_ELEVATOR_GO_TO, -1,-220),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            new AutonWaypoint(new Point(-16, 15.4), .45, DRIVE_TO, 1, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-22.5, 14.9), .37, DRIVE_TO, 1, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(LOCK_WHEELS)
    ),
    AUTON_PLACE_2_HP_BLUE_NO_LEVEL(
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 180),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT100),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 180, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 180,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 180,   ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 180,  ARM_ELEVATOR_GO_TO, 0,-225),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            //new AutonWaypoint(new Point(-17, 14.2), .5, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-33),,
            new AutonWaypoint(new Point(-54 + 16, 15), .33, DRIVE_TO, 180, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-54 + 21.5, 15), .20, DRIVE_TO, 180, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(new Point(-54 + 6.5, 14.3), .36, DRIVE_TO, 180,  ARM_ELEVATOR_GO_TO, -1,-220),
            new AutonWaypoint(INTAKE_WHEEL_IN),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    AUTON_PLACE_1_HP_BLUE_NO_LEVEL(
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 179),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(WAIT100),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 179, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 179,   ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-54 + 3.5, 15.2), .25, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, 0,-225),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            //new AutonWaypoint(new Point(-17, 14.2), .5, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -35,-33),,
            new AutonWaypoint(new Point(-54 + 16, 14.2), .33, DRIVE_TO, 180, ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-54 + 22.5, 14.2), .20, DRIVE_TO, 180, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(LOCK_WHEELS)
    ),
    AUTON_PLACE_1_ANTIHP_BLUE_LEAVE(
            new AutonWaypoint(new Point(-50.8, 1.5), .25, DRIVE_TO, 179),
            new AutonWaypoint(INTAKE_PISTON_IN),
            new AutonWaypoint(new Point(-50.8, 1.5), .25, DRIVE_TO, 179, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-50.8, 1.5), .25, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-50.8, 1.5), .25, DRIVE_TO, 179, ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-50.8, 1.5), .25, DRIVE_TO, 179,  ARM_ELEVATOR_GO_TO, -1,-240),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            new AutonWaypoint(new Point(-54 + 16, 4), .4, DRIVE_TO, 180,  ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-54 + 21.5, 4), .4, DRIVE_TO, 180, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    AUTON_PLACE_1_ANTIHP_RED_LEAVE(
            new AutonWaypoint(new Point(-3.2, 1.5), .25, INTAKE_PISTON_IN),
            new AutonWaypoint(new Point(-3.2, 1.5), .25, DRIVE_TO, 1, ARM_ELEVATOR_UP),
            new AutonWaypoint(new Point(-3.2, 1.5), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, 2,-60),
            new AutonWaypoint(new Point(-3.2, 1.5), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -3,-207),
            new AutonWaypoint(new Point(-3.2, 1.5), .25, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -1,-240),
            new AutonWaypoint(INTAKE_PISTON_OUT),
            new AutonWaypoint(INTAKE_WHEEL_OUT),
            new AutonWaypoint(new Point(-16, 3.55), .45, DRIVE_TO, 1,  ARM_ELEVATOR_GO_TO, -37,-30),
            new AutonWaypoint(new Point(-21, 3.55), .45, DRIVE_TO, 1, DRIVE_TO_CUBE),
            new AutonWaypoint(INTAKE_WHEEL_OFF),
            new AutonWaypoint(LOCK_WHEELS)

    ),
    AUTON_LEAVE_RED(
        new AutonWaypoint(new Point(3.2, 1.5), .1, DRIVE_TO,3),
        new AutonWaypoint(new Point(18, 1.5), .1, DRIVE_TO,3)
    )
    ;

    public static SendableChooser<AutonRoutines> myChooser;
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