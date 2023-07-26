package frc.misc;

import edu.wpi.first.wpilibj.*;

import static frc.robot.Robot.robotSettings;

/**
 * I accidentally deleted this, so here we go again. Allows you to control all
 * of the solenoids for all of your air
 * powered needs (pnoomatics)
 *
 * @author Smaltin
 */
public class Pneumatics implements ISubsystem {
    public DoubleSolenoid intakePiston;
    public DoubleSolenoid spikePiston;

    public DoubleSolenoid clawPiston;

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
            intakePiston = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE,
                    robotSettings.INTAKE_OUT_ID, robotSettings.INTAKE_IN_ID);
        }
        if (robotSettings.ENABLE_SPIKE && robotSettings.ENABLE_PNOOMATICS)
            spikePiston = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE,
                    robotSettings.SPIKE_OUT_ID, robotSettings.SPIKE_IN_ID);
        if (robotSettings.ENABLE_CLAW && robotSettings.ENABLE_PNOOMATICS)
            clawPiston = new DoubleSolenoid(robotSettings.PCM_ID, robotSettings.PNEUMATICS_MODULE_TYPE,
                    robotSettings.CLAW_OUT_ID, robotSettings.CLAW_IN_ID);
        
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
            System.out.println("---------------- Running Compressor of type CTREPCM");
            compressor.enableDigital();

        } else if (robotSettings.PNEUMATICS_MODULE_TYPE == PneumaticsModuleType.REVPH) {
            System.out.println("---------------- Running Compressor of type REVPH");
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
    }

    @Override
    public void initGeneric() {
    }

    @Override
    public String getSubsystemName() {
        return "Pneumatics";
    }
}
