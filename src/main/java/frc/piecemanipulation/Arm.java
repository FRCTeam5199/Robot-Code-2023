package frc.piecemanipulation;

import frc.controllers.basecontrollers.BaseController;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;

public class Arm implements ISubsystem {
    public AbstractMotorController arm;
    public BaseController xbox;

    public Arm(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {

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
}
