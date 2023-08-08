package frc.piecemanipulation;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.controllers.basecontrollers.DefaultControllerEnums.ButtonStatus;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.robot.Robot;

import static frc.robot.Robot.*;

public class Claw implements ISubsystem {
    // public AbstractMotorController clawLeft, clawRight, clawBottom;
    private BaseController xbox, panel1, panel2, midiTop, midiBot;
    private I2C.Port i2cPort;
    public ColorSensorV3 m_colorSensor;
    public Timer closeTimer;

    public Claw() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        // createMotors();
        closeTimer = new Timer();
        closeTimer.reset();
        closeTimer.start();
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
        if (robotSettings.CLAW_MANUAL)
            manuelDrive();
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        updateGeneric();
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
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3, BaseController.DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void manuelDrive() {
        if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
            // System.out.println("Left Claw");
            Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kReverse);
        } else if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            // System.out.println("Right Claw");
            Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kForward);
        }
    }
}