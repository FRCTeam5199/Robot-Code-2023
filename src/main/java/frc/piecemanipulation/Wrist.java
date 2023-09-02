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
    // USE PID VALUE FOR COMPETITION: 0.01 \/
    public PIDController wristPIDController = new PIDController(0.003, 0, 0);
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

        wristPIDController.setTolerance(5, 10);
        // wristController.setCurrentLimit(10);

        wristController.resetEncoder();
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
            wristController = new SparkMotorController(robotSettings.WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushed);
        // wrist.setCurrentLimit(2,40);
        wristController.setCurrentLimit(40);
        wristController.setRealFactorFromMotorRPM(1, 1);

        wristController.setBrake(true);
    }

    public void createControllers() {
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void moveWrist() {
        if (wristController.getRotations() > 1000) {
            System.out.println("Readjusting Wrist...");
            wristController.moveAtPercent(-1);
        }

        if (xbox.get(DefaultControllerEnums.XBoxButtons.GUIDE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            wristPIDController.setSetpoint(330);
            System.out.println("Right Wrist: " + wristController.getRotations());
            System.out.println("PID TARGET RIGHT WRIST PERCENT: " + wristPIDController.calculate(wristController.getRotations()));
        } else if (xbox.get(DefaultControllerEnums.XBoxButtons.MENU) ==
            DefaultControllerEnums.ButtonStatus.DOWN) {
            wristPIDController.setSetpoint(30);
            System.out.println("Left Wrist: " + wristController.getRotations());
            System.out.println("PID TARGET LEFT WRIST PERCENT: " + wristPIDController.calculate(wristController.getRotations()));
        }

        System.out.println("Wrist: " + wristController.getRotations());
        System.out.println("PID TARGET WRIST PERCENT: " + -wristPIDController.calculate(wristController.getRotations()));

        wristController.moveAtPercent(-wristPIDController.calculate(wristController.getRotations()));
    }
}
