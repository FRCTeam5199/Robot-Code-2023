package frc.piecemanipulation;

import static frc.robot.Robot.robotSettings;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import static frc.robot.Robot.arm;
import static frc.robot.Robot.elevator;
import static frc.robot.Robot.intake;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.piecemanipulation.ManipulationManager;
import frc.robot.Robot;
import frc.controllers.basecontrollers.BaseController.DefaultControllers;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;

public class CompositeManager implements ISubsystem  {
    public BaseController panel1, panel2, xbox, xbox2, midiTop, midiBot;
    public boolean CubeConeMode;

    private enum stateMachine {
        HUMANPLAYER,
        STABLE,
        HIGH,
        MID,
        LOW
    }

    private stateMachine currentState = stateMachine.STABLE;

    public CompositeManager() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return null;
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {
        intake.spinBottomIntake(0);
        if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.Cone) == DefaultControllerEnums.ButtonStatus.DOWN) {
            CubeConeMode = true;
        }
        if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.Cube) == DefaultControllerEnums.ButtonStatus.DOWN) {
            CubeConeMode = false;
        }
        if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeD) == DefaultControllerEnums.ButtonStatus.DOWN) {
            if (CubeConeMode == false) {
                Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
            }
        } 
        if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeU) == DefaultControllerEnums.ButtonStatus.DOWN) {
            Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
        }
        if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == DefaultControllerEnums.ButtonStatus.DOWN) {
            Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
        }

        if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            intake.spinBottomIntake(-3);
        }
        if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
            intake.spinBottomIntake(3);
        }
        if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == DefaultControllerEnums.ButtonStatus.DOWN) {
            currentState = stateMachine.HUMANPLAYER;
            Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
        // Stable
        } else if (panel2.get(ControllerEnums.ButtonPanelButtonsPlacement2023.Stable) == DefaultControllerEnums.ButtonStatus.DOWN) {
            currentState = stateMachine.STABLE;


        // High Cone Goal
        } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.High) == DefaultControllerEnums.ButtonStatus.DOWN) {
            currentState = stateMachine.HIGH;

        // Mid Cone Goal
        } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Mid) == DefaultControllerEnums.ButtonStatus.DOWN) {
            currentState = stateMachine.MID;

        // Low Cone Goal
        } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Low) == DefaultControllerEnums.ButtonStatus.DOWN) {
            currentState = stateMachine.LOW;
        }

        switch (currentState) {
            case HUMANPLAYER:
                arm.armExtendingPIDController.setSetpoint(5);
                
                if (arm.armExtendingController.getRotations() > 3) {
                    arm.armRotationPIDController.setSetpoint(22);
                    elevator.elevatorPIDController.setSetpoint(38);
                }

                break;
            case STABLE:
                // arm.armExtendingPIDController.setSetpoint(0);
                arm.armExtendingPIDController.setSetpoint(7);

                elevator.elevatorPIDController.setSetpoint(0);
                if (arm.armExtendingController.getRotations() > 3) {
                    arm.armRotationPIDController.setSetpoint(0);
                }

                if ((arm.armRotationController.getRotations() < 9) && (arm.armRotationController.getRotations() > -9)) {
                    // System.out.println("STABLE Condition: armRotationController.getRotations() < 9");
                    arm.armExtendingPIDController.setSetpoint(2);
                }
                
                break;
            case HIGH:
                    // arm.armExtendingPIDController.setSetpoint(0);
                    arm.armExtendingPIDController.setSetpoint(7);
                
                    if (arm.armExtendingController.getRotations() > 3) {
                        arm.armRotationPIDController.setSetpoint(-64);
                    }

                    if (arm.armRotationController.getRotations() < -40) {
                        // System.out.println("HIGH Condition: armRotationController.getRotations() > 40");
                        arm.armExtendingPIDController.setSetpoint(21
                        );
                        elevator.elevatorPIDController.setSetpoint(38);
                    }

                break;
            case MID:
                // arm.armExtendingPIDController.setSetpoint(0);
                arm.armExtendingPIDController.setSetpoint(7);
                
                if (arm.armExtendingController.getRotations() > 3) {
                    arm.armRotationPIDController.setSetpoint(-60);
                }

                if (arm.armRotationController.getRotations() < -40) {
                    arm.armExtendingPIDController.setSetpoint(0);
                    elevator.elevatorPIDController.setSetpoint(16);
                }

                break;
            case LOW:
                // arm.armExtendingPIDController.setSetpoint(0);
                arm.armExtendingPIDController.setSetpoint(7);
                
                // if (arm.armExtendingController.getRotations() > 3) {
                    arm.armRotationPIDController.setSetpoint(-75);
                // }

                if (arm.armRotationController.getRotations() < -40) {
                    // System.out.println("LOW Condition: armRotationController.getRotations() > 40");
                    arm.armExtendingPIDController.setSetpoint(0);
                    elevator.elevatorPIDController.setSetpoint(0);
                }

                break;
            }

        if (((arm.armRotationController.getRotations() > -45) && (arm.armRotationController.getRotations() < 0)) && arm.armExtendingController.getRotations() < 6) {
            // System.out.println("Extending...");
            arm.armExtendingPIDController.setSetpoint(7);
        }
        
        System.out.println(arm.armRotationController.getRotations());
    }

    @Override   
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {

    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initAuton() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    @Override
    public String getSubsystemName() {
        return null;
    }

    public void createControllers() {
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3, BaseController.DefaultControllers.BUTTON_PANEL);
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
    }
}
