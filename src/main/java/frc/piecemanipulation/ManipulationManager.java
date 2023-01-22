package frc.piecemanipulation;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.robot.Robot;

import static frc.robot.Robot.robotSettings;


public class ManipulationManager implements ISubsystem {
    public BaseController panel;
    public int armGoal = 0;
    public int elevateGoal = 0;

    public ManipulationManager(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        enableControllers();
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
        if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_3) == DefaultControllerEnums.ButtonStatus.DOWN) {
            Robot.arm.moveArm(-15);
            elevateGoal = -5;
        }
        if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_2) == DefaultControllerEnums.ButtonStatus.DOWN) {
            Robot.arm.moveArm(-90);
            elevateGoal = -5;
        }
        if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == DefaultControllerEnums.ButtonStatus.DOWN) {
            Robot.arm.moveArm(0);
            elevateGoal = 0;
        }
        if(!robotSettings.ARM_MANUAL) {
            if (checkArmCollision()) {
            } else if (checkArmPassover()) {
                Robot.elevator.moveElevator(-50);
            } else {
                    Robot.elevator.moveElevator(elevateGoal);
            }
        }
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

    public void enableControllers(){
        panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public boolean checkArmPassover(){
        if (-25 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -69){
            return true;
        }
        return false;
    }

    public boolean checkArmCollision(){
        if(Robot.arm.arm.getRotations() >= -15 || Robot.arm.arm.getRotations() <= -70){
           return true;
        }
        return false;
    }
}
