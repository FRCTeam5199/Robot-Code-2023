package frc.selfdiagnostics;

import frc.misc.ISubsystem;
import frc.motors.AbstractMotorController;
import frc.motors.followers.AbstractFollowerMotorController;
import frc.robot.Main;

import static frc.robot.Robot.robotSettings;

/**
 * Pretty self explanatory. This issue is regarding an non operational Motor
 */
public class MotorDisconnectedIssue implements ISimpleIssue {
    private static final String[] RANDOM_FIXES = {"Ensure there is a motor with an id of %d", "Ensure motor with id %d is plugged in", "Ensure motor %d can spin freely", "Verify motor %d is not boiling"};
    private final int faultedMotorID;
    private final boolean knownFix;
    private final String fix;

    public static void handleIssue(ISubsystem owner, AbstractFollowerMotorController... motors) {
        for (AbstractFollowerMotorController motor : motors)
            if (motor.failureFlag())
                reportIssue(owner, motor.getBundleID(), motor.getSuggestedFix());
            else
                resolveIssue(owner, motor.getBundleID());
    }

    private static void reportIssue(ISubsystem owner, int id, String fix) {
        if (!IssueHandler.issues.containsKey(owner) || !(IssueHandler.issues.get(owner) instanceof MotorDisconnectedIssue)) {
            IssueHandler.issues.put(owner, new MotorDisconnectedIssue(id, fix));
        }
    }

    private static void resolveIssue(ISubsystem owner, int fixedMotorID) {
        if (IssueHandler.issues.get(owner) instanceof MotorDisconnectedIssue)
            if (((MotorDisconnectedIssue) IssueHandler.issues.get(owner)).faultedMotorID == fixedMotorID) {
                IssueHandler.issues.remove(owner);
            }
    }

    public static void handleIssue(ISubsystem owner, AbstractMotorController... motors) {
        for (AbstractMotorController motor : motors)
            if (motor != null)
                if (motor.isFailed())
                    reportIssue(owner, motor.getID(), motor.getSuggestedFix());
                else
                    resolveIssue(owner, motor.getID());
    }

    private MotorDisconnectedIssue(int motorID, String theFix) {
        faultedMotorID = motorID;
        knownFix = !theFix.equals("");
        fix = theFix;
    }

    @Override
    public String getRandomFix() {
        return knownFix ? fix : "Not sure, " + String.format(RANDOM_FIXES[Main.RANDOM.nextInt(RANDOM_FIXES.length)], faultedMotorID);
    }
}
