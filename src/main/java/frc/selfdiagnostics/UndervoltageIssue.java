package frc.selfdiagnostics;



import frc.misc.ISubsystem;
import frc.robot.Main;

import static frc.robot.Robot.robotSettings;

/**
 * Not a {@link BrownoutIssue}. Activates sooner, is resolvable and does not use {@link frc.gpws.Alarms} but rather a
 * standard {@link frc.robot.ClientServerPipeline#sendSound(Sound)} to pass a simple message
 */
public class UndervoltageIssue implements ISimpleIssue {
    public static void handleIssue(ISubsystem owner, boolean report) {
        if (!report)
            resolveIssue(owner);
        else
            reportIssue(owner);
    }

    private static void resolveIssue(ISubsystem owner) {
        if (IssueHandler.issues.get(owner) instanceof UndervoltageIssue) {
            IssueHandler.issues.remove(owner);
        }
    }

    private static void reportIssue(ISubsystem owner) {
        if (!IssueHandler.issues.containsKey(owner) || !(IssueHandler.issues.get(owner) instanceof UndervoltageIssue)) {
            System.err.println(">>>>>>>POSSIBLE BROWNOUT DETECTED<<<<<<<");
            IssueHandler.issues.put(owner, new UndervoltageIssue());
        }
    }

    private UndervoltageIssue() {

    }

    @Override
    public String getRandomFix() {
        return "Replace the battery or turn off unused subsystems";
    }
}
