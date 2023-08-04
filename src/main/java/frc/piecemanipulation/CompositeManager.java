package frc.piecemanipulation;

import static frc.robot.Robot.robotSettings;
import static frc.robot.Robot.arm;
import static frc.robot.Robot.elevator;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;

public class CompositeManager implements ISubsystem  {
    public BaseController panel1, panel2, xbox2, midiTop, midiBot;

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
        // if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     currentState = stateMachine.HUMANPLAYER;
        // // Stable
        // } else if (panel2.get(ControllerEnums.ButtonPanelButtonsPlacement2023.Stable) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     currentState = stateMachine.STABLE;

        // // High Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.High) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     currentState = stateMachine.HIGH;

        // // Mid Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Mid) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     currentState = stateMachine.MID;

        // // Low Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Low) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     currentState = stateMachine.LOW;
        // }

        // switch (currentState) {
        //     case HUMANPLAYER:
        //         arm.armRotationController.setInverted(false);
        //         arm.armRotationPIDController.setSetpoint(27);

        //         elevator.elevatorPIDController.setSetpoint(38);
                
        //         break;
        //     case STABLE:
        //         arm.armRotationController.setInverted(true);
        //         arm.armRotationPIDController.setSetpoint(5);
        //         elevator.elevatorPIDController.setSetpoint(0);

        //         break;
        //     case HIGH:
        //         arm.armRotationController.setInverted(true);
        //         elevator.elevatorPIDController.setSetpoint(0);
        //         arm.armRotationPIDController.setSetpoint(63);

        //         if (arm.armRotationController.getRotations() > 40) {
        //             elevator.elevatorPIDController.setSetpoint(38);
        //         }

        //         break;
        //     case MID:
        //         arm.armRotationController.setInverted(true);
        //         elevator.elevatorPIDController.setSetpoint(0);
        //         arm.armRotationPIDController.setSetpoint(88);

        //         if (arm.armRotationController.getRotations() > 40) {
        //             elevator.elevatorPIDController.setSetpoint(38);
        //         }

        //         break;
        //     case LOW:
        //         arm.armRotationController.setInverted(true);
        //         elevator.elevatorPIDController.setSetpoint(0);
        //         arm.armRotationPIDController.setSetpoint(55);

        //         break;
        // }
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
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2,
                BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3,
                BaseController.DefaultControllers.BUTTON_PANEL);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2,
                BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
    }
}
