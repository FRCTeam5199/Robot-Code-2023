package frc.misc;

import edu.wpi.first.wpilibj.*;

import static frc.robot.Robot.robotSettings;

/**
 * I accidentally deleted this, so here we go again. Allows you to control all of the solenoids for all of your air
 * powered needs (pnoomatics)
 *
 * @author Smaltin
 */
public class Pneumatics implements ISubsystem {
    public DoubleSolenoid solenoidIntake;
    public DoubleSolenoid subsolenoidIntake;

    public DoubleSolenoid climberLock;
    public DoubleSolenoid indexerBlocker;
    public Compressor compressor;
    public PneumaticHub pneumaticsHub;

    public Pneumatics() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        if (robotSettings.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.REVPH) {
            pneumaticsHub = new PneumaticHub(robotSettings.PCM_ID);
            pneumaticsHub.clearStickyFaults();
        } else {
            compressor = new Compressor(robotSettings.PNEUMATICS_MODULE_TYPE);
        }
        if (robotSettings.ENABLE_INTAKE && robotSettings.ENABLE_PNOOMATICS) {
            solenoidIntake = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.INTAKE_OUT_ID, robotSettings.INTAKE_IN_ID);
            subsolenoidIntake = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE, robotSettings.INTAKE_OUT_ID, robotSettings.INTAKE_IN_ID);

        }

    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public void updateTest() {
        updateGeneric();
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        updateGeneric();
    }

    @Override
    public void updateGeneric() {
        if (robotSettings.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.CTREPCM) {
            compressor.enableDigital();

        } else if (robotSettings.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.REVPH) {
                pneumaticsHub.enableCompressorDigital();
        }
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {
        initGeneric();
    }

    @Override
    public void initAuton() {
        initGeneric();
    }

    @Override
    public void initDisabled() {
        if (robotSettings.ENABLE_PNOOMATICS)
            indexerBlocker.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void initGeneric() {
        if (robotSettings.ENABLE_PNOOMATICS)
            climberLock.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public String getSubsystemName() {
        return "Pneumatics";
    }
}
