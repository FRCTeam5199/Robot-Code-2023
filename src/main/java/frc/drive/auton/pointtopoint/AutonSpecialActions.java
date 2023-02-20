package frc.drive.auton.pointtopoint;

/**
 * Used by {@link AutonManager Point to Point} to do cool things with the robot. Processed in {@link
 * AutonManager#updateAuton()}.
 *
 * @author Smaltin
 */
public enum AutonSpecialActions {
    /**
     * You're all good, do nothing
     */
    NONE,
    /**
     * Intarg will be the degrees rotation
     */
    ROTATE_ROBOT,
    /**
        Intarg will not be used
     */
    AUTO_LEVEL,
    /**
      Uses intarg 2 and 3 in the special action 2
     */
    ARM_ELEVATOR_GO_TO,
    /**
        Dousnt use the Intarf
     */
    ARM_ELEVATOR_UP,
    ARM_ELEVATOR_DOWN,
    CUBE_TOP,
    /**
     Intarg will not be used
     locks swerve wheels at a 45 degree angle
     */
    LOCK_WHEELS,

    INTAKE_PISTON_IN,

    INTAKE_PISTON_OUT,

    INTAKE_WHEEL_IN,

    INTAKE_WHEEL_OUT,

    INTAKE_WHEEL_OFF,

    WAIT1000,
    WAIT500,
    WAIT100,
    /**
     * If intarg is != 0, will swerve, otherwise will west coast
     */
    DRIVE_TO, ARM_ELEVATOR_RESET;
}