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
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;
import frc.motors.VictorMotorController;
import frc.robot.Robot;
import frc.sensors.colorsensor.RevColorSensor;

import java.sql.Time;

import static frc.robot.Robot.*;


public class Intake implements ISubsystem {
    public AbstractMotorController intakeLeft, intakeRight;
    private BaseController xbox, panel1, panel2, midiTop, midiBot;
    private I2C.Port i2cPort;
    public ColorSensorV3 m_colorSensor;
    public Timer closeTimer;

    public Intake() {
        addToMetaList();
        init();
    }
    @Override
    public void init() {
        createControllers();
        createMotors();
        if (robotSettings.ENABLE_COLOR_SENSOR){
            i2cPort = I2C.Port.kMXP;
            m_colorSensor = new ColorSensorV3(i2cPort);
        }
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
        if (robotSettings.INTAKE_MANUAL)
            manuelDrive();
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        updateGeneric();
    }

    @Override
    public void updateGeneric() {
        if (robotSettings.ENABLE_COLOR_SENSOR){
            UserInterface.smartDashboardPutNumber("proxy cube", m_colorSensor.getProximity());
            if(m_colorSensor.getProximity() >= 615){
                if (intakeLeft.getVoltage() >= 10 || intakeRight.getVoltage() >= 10){
                    intakeLeft.moveAtVoltage(0);
                    intakeRight.moveAtVoltage(0);
                }
            }
        }
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

    public void createMotors(){
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX) {
            intakeLeft = new TalonMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID, robotSettings.INTAKE_MOTOR_CANBUS);
            intakeRight = new TalonMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID, robotSettings.INTAKE_MOTOR_CANBUS);
        }
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX) {
            intakeLeft = new SparkMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID);
            intakeRight = new SparkMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID);
        }
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.VICTOR) {
            intakeLeft = new VictorMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID);
            intakeRight = new VictorMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID);
        }
        intakeLeft.setCurrentLimit(20);
        intakeRight.setCurrentLimit(20);
        intakeRight.setBrake(true);
        intakeLeft.setBrake(true);
    }

    public void createControllers(){
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void manuelDrive(){
        if(!robotSettings.BRANDONISNOTHERE) {
            if (!manipulationManager.cubeConeMode) {
                if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    //System.out.println("X is being pressed");
                    intakeRight.moveAtVoltage(3);
                    intakeLeft.moveAtVoltage(-3);
                } else if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    if(robotSettings.ENABLE_COLOR_SENSOR) {
                        if (m_colorSensor.getProximity() >= 400) {
                            intakeLeft.moveAtVoltage(0);
                            intakeRight.moveAtVoltage(0);
                        }else {
                            //System.out.println("Y is being pressed");
                            intakeRight.moveAtVoltage(-12);
                            intakeLeft.moveAtVoltage(12);
                        }
                    }else {
                        //System.out.println("Y is being pressed");
                        intakeRight.moveAtVoltage(-12);
                        intakeLeft.moveAtVoltage(12);
                    }
                } else {
                    intakeRight.moveAtVoltage(0);
                    intakeLeft.moveAtVoltage(0);
                }
            }
            if (manipulationManager.cubeConeMode) {
                //System.out.println(elevate.getRotations());
                intakeRight.moveAtVoltage(0);
                intakeLeft.moveAtVoltage(0);
                if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
                    closeTimer.reset();
                }
                if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
                    closeTimer.reset();
                }
                if (robotSettings.ENABLE_COLOR_SENSOR){
                    if (m_colorSensor.getProximity() >= 150 && closeTimer.get() >= .5) {
                        Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
                        closeTimer.reset();
                    }
                }
            }
        }else{
            if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                //System.out.println("X is being pressed");
                intakeRight.moveAtVoltage(6);
                intakeLeft.moveAtVoltage(-6);
            } else if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
                if(robotSettings.ENABLE_COLOR_SENSOR) {
                    if (m_colorSensor.getProximity() >= 615) {
                            intakeLeft.moveAtVoltage(0);
                            intakeRight.moveAtVoltage(0);
                    }else {
                        //System.out.println("Y is being pressed");
                        intakeRight.moveAtVoltage(-12);
                        intakeLeft.moveAtVoltage(12);
                    }
                }else {
                    //System.out.println("Y is being pressed");
                    intakeRight.moveAtVoltage(-12);
                    intakeLeft.moveAtVoltage(12);
                }
            } else {
                intakeRight.moveAtVoltage(0);
                intakeLeft.moveAtVoltage(0);
            }
            if (xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
                closeTimer.reset();
            }
            if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
                closeTimer.reset();
            }
            if (closeTimer.hasElapsed(.5)){
                if (m_colorSensor.getProximity() >= 1024) {
                    Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
                    closeTimer.reset();
                }
            }
        }
        if(!robotSettings.ARM_ELEVATOR_MANUAL){
            switch (robotSettings.DRIVE_STYLE) {
                case MIDI: {
                    if(midiTop.get(ControllerEnums.MidiController.R2C4) == DefaultControllerEnums.ButtonStatus.DOWN){
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if(midiTop.get(ControllerEnums.MidiController.R1C3) == DefaultControllerEnums.ButtonStatus.DOWN){
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if(midiTop.get(ControllerEnums.MidiController.R1C2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                    }
                    break;
                }  
                case STANDARD_2023:{
                    if(panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == ButtonStatus.DOWN){
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if(panel1.get(ControllerEnums.ButtonPanelButtonsPlacement2023.Stable) == ButtonStatus.DOWN){
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                    }
                    if(panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTShute) == ButtonStatus.DOWN){
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if(panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeD) == ButtonStatus.DOWN){
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if(panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeU) == ButtonStatus.DOWN){
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                        break;
                    }
                    break;
                }
            }
            //}
        }else {
            switch (robotSettings.DRIVE_STYLE) {
                case MIDI: {
                    if (midiTop.get(ControllerEnums.MidiController.R2C5) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (midiTop.get(ControllerEnums.MidiController.R2C6) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                    }
                    break;
                }
                case STANDARD_2023:{
                    if (panel2.get(ControllerEnums.MidiController.R2C5) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (panel2.get(ControllerEnums.MidiController.R2C6) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                        break;
                    }
                }
            }
        }



    }
    public void intakeIn(){
        Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
    }
    public void intakeOut(){
        Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
    }
}