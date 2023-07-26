package frc.piecemanipulation;

import static frc.robot.Robot.robotSettings;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;

public class Wrist implements ISubsystem {
    public AbstractMotorController wristController;
    public PIDController wristPIDController = new PIDController(0.01, 0, 0);
    public BaseController panel1, panel2, xbox, xbox2, midiTop, midiBot;

    public Wrist() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        createMotors();
        wristController.setPid(robotSettings.WRISTPID);
        wristController.setCurrentLimit(10);
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
        /*
         * if(robotSettings.ARM_ELEVATOR_MANUAL){
         * if (midiTop.get(ControllerEnums.MidiController.R3C7) ==
         * DefaultControllerEnums.ButtonStatus.DOWN) {
         * wrist.moveAtVoltage(-2);
         * }
         * else if (midiTop.get(ControllerEnums.MidiController.R2C8) ==
         * DefaultControllerEnums.ButtonStatus.DOWN) {
         * wrist.moveAtVoltage(2);
         * }
         * else {
         * wrist.moveAtVoltage(0);
         * }
         * }else {
         * if (midiTop.get(ControllerEnums.MidiController.R2C7) ==
         * DefaultControllerEnums.ButtonStatus.DOWN) {
         * wrist.moveAtPosition(-2);
         * }
         * else if (midiTop.get(ControllerEnums.MidiController.R2C8) ==
         * DefaultControllerEnums.ButtonStatus.DOWN) {
         * wrist.moveAtPosition(-40.5);
         * }
         * }
         */
        // System.out.println(wrist.getRotations());
        if (xbox2.get(DefaultControllerEnums.XBoxButtons.RIGHT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN)
            wristController.resetEncoder();
        // System.out.println(wrist.getRotations());
        moveWrist();
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

    public void createMotors() {
        if (robotSettings.WRIST_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            wristController = new TalonMotorController(robotSettings.WRIST_MOTOR_ID, robotSettings.WRIST_MOTOR_CANBUS);
        if (robotSettings.WRIST_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            wristController = new SparkMotorController(robotSettings.WRIST_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless);
        // wrist.setCurrentLimit(2,40);
        wristController.setCurrentLimit(40);
        wristController.setBrake(true);
    }

    public void createControllers() {
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT,
                BaseController.DefaultControllers.XBOX_CONTROLLER);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2,
                BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1,
                BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2,
                BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void moveWrist() {
        // FIND BETTER BUTTONS FOR WRIST \/
        // MAKE IT SO WHEN YOU PRESS BUTTON ONCE IT WILL FLIP WRIST \/

        // if (xbox.get(DefaultControllerEnums.XBoxButtons.MENU) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     System.out.println("Right Wrist: " + wristController.getRotations());
        //     System.out.println("PID TARGET RIGHT WRIST PERCENT: "
        //             + wristPIDController.calculate(wristController.getRotations(), 5));
        //     wristController.moveAtPercent(wristPIDController.calculate(wristController.getRotations(), 5));
        // }

        // else if (xbox.get(DefaultControllerEnums.XBoxButtons.GUIDE) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     System.out.println("Left Wrist: " + wristController.getRotations());
        //     System.out.println("PID TARGET LEFT WRIST PERCENT: "
        //             + wristPIDController.calculate(wristController.getRotations(), 0));
        //     wristController.moveAtPercent(wristPIDController.calculate(wristController.getRotations(), 0));
        // } else {
        //     wristController.moveAtPercent(0);
        // }
    }
}
