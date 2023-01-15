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
     * If intarg is != 0, will swerve, otherwise will west coast
     */
    DRIVE_TO;
}