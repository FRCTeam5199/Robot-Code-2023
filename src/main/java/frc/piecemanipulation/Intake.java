package frc.piecemanipulation;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.robotconfigs.SwervePrac2023;
import frc.sensors.colorsensor.RevColorSensor;

import java.sql.Time;

import static frc.robot.Robot.*;


public class Intake implements ISubsystem {
    // public AbstractMotorController intakeLeft, intakeRight, intakeBottom;
    public AbstractMotorController intakeBottom;
    private BaseController xbox, panel1, panel2, midiTop, midiBot;
    private I2C.Port i2cPort;
    public ColorSensorV3 m_colorSensor;
    public Timer closeTimer;
    public DoublePublisher bottomIntakeVoltagePub;


    public Intake() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("bottomIntake");
        bottomIntakeVoltagePub = table.getDoubleTopic("bottomIntake").publish();

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
            // if(m_colorSensor.getProximity() >= 350){
            //     if (intakeLeft.getVoltage() >= 10 || intakeRight.getVoltage() >= 10){
            //         intakeLeft.moveAtVoltage(0);
            //         intakeRight.moveAtVoltage(0);
            //     }
            // }
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
          
            // intakeLeft = new TalonMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID, robotSettings.INTAKE_MOTOR_CANBUS);
            // intakeRight = new TalonMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID, robotSettings.INTAKE_MOTOR_CANBUS);
        }
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX) {
            // intakeLeft = new SparkMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID);
            // intakeRight = new SparkMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID);
            intakeBottom = new SparkMotorController(robotSettings.INTAKE_MOTOR_BOTTOM_ID, MotorType.kBrushed);
        }
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.VICTOR) {
            // intakeLeft = new VictorMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID);
            // intakeRight = new VictorMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID);
        }
        if(robotSettings.INTAKE_MOTOR_BOTTOM_TYPE == AbstractMotorController.SupportedMotors.VICTOR){
            intakeBottom = new VictorMotorController(robotSettings.INTAKE_MOTOR_BOTTOM_ID);
        }

        // intakeLeft.setCurrentLimit(20);
        // intakeRight.setCurrentLimit(20);
        // intakeRight.setBrake(true);
        // intakeLeft.setBrake(true);
        // intakeRight.setInverted(true);
        // intakeLeft.setInverted(true);
        intakeBottom.setBrake(true);
        intakeBottom.setInverted(false);

    }

    public void createControllers() {
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT,
                BaseController.DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2,
                BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3,
                BaseController.DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void manuelDrive() {
        if(manipulationManager.cubeConeMode) { //cone
            if(xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == ButtonStatus.DOWN){
                // intakeRight.moveAtVoltage(-12);
                // intakeLeft.moveAtVoltage(12);
                intakeBottom.moveAtPercent(0);
            }
        }
        
        if(!manipulationManager.cubeConeMode) { //cube
            if(xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == ButtonStatus.DOWN){ //if x pressed
                boolean stopped = false;
                if (intakeBottom.getCurrent() > 0.65 && !stopped) { 
                    // checks for current spike and if stopped is false, motor is set to zero and all bottom motor movement is disabled until x is released :)
                    intakeBottom.moveAtPercent(0);
                    stopped = true;
                }
                if (robotSettings.ENABLE_COLOR_SENSOR) { // color sensor on 
                    if (m_colorSensor.getProximity() >= 350) {
                        // intakeRight.moveAtVoltage(0);
                        // intakeLeft.moveAtVoltage(0);
                        intakeBottom.moveAtPercent(0); 
                    } 
                    else{
                        // intakeRight.moveAtVoltage(-12);
                        // intakeLeft.moveAtVoltage(12);
                        if (!stopped){ // if stopped is false it spins
                            intakeBottom.moveAtPercent(-.6);
                        }
                    }
                }

                if (!robotSettings.ENABLE_COLOR_SENSOR) { //color sensor off
                    // intakeRight.moveAtPercent(12);
                    // intakeLeft.moveAtPercent(12);
                    if (!stopped){ // if stopped is false it spins
                        intakeBottom.moveAtPercent(-.6);
                    }
                }
            }
        }

        if (!robotSettings.ARM_ELEVATOR_MANUAL) { // if false
            switch (robotSettings.DRIVE_STYLE){
                case STANDARD_2023: {
                if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == ButtonStatus.DOWN) {
                    pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                }
                if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == ButtonStatus.DOWN) {
                    Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                }
                if (panel1.get(ControllerEnums.ButtonPanelButtonsPlacement2023.Stable) == ButtonStatus.DOWN) {
                    Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                }
                if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTShute) == ButtonStatus.DOWN) {
                    Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                }
                if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeD) == ButtonStatus.DOWN) {
                    Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                }
                if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeU) == ButtonStatus.DOWN) {
                    Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                }
                break;
                }
                default:
                    break;
            }
        }
        else { // if true
            switch (robotSettings.DRIVE_STYLE){
                case STANDARD_2023: {
                      if (panel2.get(ControllerEnums.MidiController.R2C5) ==
                      DefaultControllerEnums.ButtonStatus.DOWN) {
                      Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                      }
                      if (panel2.get(ControllerEnums.MidiController.R2C6) ==
                      DefaultControllerEnums.ButtonStatus.DOWN) {
                      Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                      break;
                      } 
                }
                default:
                    break;
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
    
    


        // if (!robotSettings.BRANDONISNOTHERE) {
        //     if (!manipulationManager.cubeConeMode) {
        //         if (xbox.get(
        //                 DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //             // System.out.println("X is being pressed");
        //             intakeRight.moveAtVoltage(3);
        //             intakeLeft.moveAtVoltage(-3);
        //             intakeBottom.moveAtPercent(1);
        //         } else if (xbox
        //                 .get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //             if (robotSettings.ENABLE_COLOR_SENSOR) {
        //                 if (m_colorSensor.getProximity() >= 350) {
        //                     intakeLeft.moveAtVoltage(0);
        //                     intakeRight.moveAtVoltage(0);
        //                     intakeBottom.moveAtPercent(0);
        //                 } else {
        //             
        //                     intakeRight.moveAtVoltage(-12);
        //                     intakeLeft.moveAtVoltage(12);
        //                     intakeBottom.moveAtPercent(-.6);
        //                 }
        //             } else {
        //                 
        //                 intakeRight.moveAtVoltage(-12);
        //                 intakeLeft.moveAtVoltage(12);
        //                 intakeBottom.moveAtPercent(-.6);
        //             }
        //         } else {
        //             intakeRight.moveAtVoltage(0);
        //             intakeLeft.moveAtVoltage(0);
        //             intakeBottom.moveAtPercent(0);
        //         }
        //     }
        //     if (manipulationManager.cubeConeMode) {
        //         // System.out.println(elevate.getRotations());
        //         intakeRight.moveAtVoltage(0);
        //         intakeLeft.moveAtVoltage(0);
        //         intakeBottom.moveAtPercent(0);
        //         if (xbox.get(
        //                 DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //             Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
        //             closeTimer.reset();
        //         }
        //         if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //             Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
        //             closeTimer.reset();
        //         }
        //         if (robotSettings.ENABLE_COLOR_SENSOR) {
        //             // if (m_colorSensor.getProximity() >= 150 && closeTimer.get() >= .5) {
        //             // Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
        //             // closeTimer.reset();
        //             // }
        //         }
        //     }
        // } else {
        //     if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //         intakeRight.moveAtVoltage(6);
        //         intakeLeft.moveAtVoltage(-6);
        //         intakeBottom.moveAtPercent(1);
        //     } else if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //         if (robotSettings.ENABLE_COLOR_SENSOR) {
        //             if (m_colorSensor.getProximity() >= 500) {
        //                 intakeLeft.moveAtVoltage(0);
        //                 intakeRight.moveAtVoltage(0);
        //                 intakeBottom.moveAtPercent(0);
        //             } else {
        //                
        //                 intakeRight.moveAtVoltage(-12);
        //                 intakeLeft.moveAtVoltage(12);
        //                 intakeBottom.moveAtPercent(-.6);
        //             }
        //         } else {
        //             
        //             intakeRight.moveAtVoltage(-12);
        //             intakeLeft.moveAtVoltage(12);
        //             intakeBottom.moveAtPercent(-.6);
        //         }
        //     } else {
        //         intakeRight.moveAtVoltage(0);
        //         intakeLeft.moveAtVoltage(0);
        //         intakeBottom.moveAtPercent(0);
        //     }
        //     if (xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //         Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
        //         closeTimer.reset();
        //     }
        //     if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //         Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
        //         closeTimer.reset();
        //     }
        //     if (closeTimer.hasElapsed(.5)) {
        //         if (m_colorSensor.getProximity() >= 1024) {
        //             Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
        //             closeTimer.reset();
        //         }
        //     }
        // }
        // if (!robotSettings.ARM_ELEVATOR_MANUAL) {
        //     switch (robotSettings.DRIVE_STYLE) {
        //         case MIDI: {
        //             if (midiTop.get(ControllerEnums.MidiController.R2C4) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //                 Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
        //             }
        //             if (midiTop.get(ControllerEnums.MidiController.R1C3) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //                 Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
        //             }
        //             if (midiTop.get(ControllerEnums.MidiController.R1C2) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //                 Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
        //             }
        //             break;
        //         }
                // case STANDARD_2023: {
                //     if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == ButtonStatus.DOWN) {
                //         pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
                //     }
                //     if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == ButtonStatus.DOWN) {
                //         Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                //     }
                //     if (panel1.get(ControllerEnums.ButtonPanelButtonsPlacement2023.Stable) == ButtonStatus.DOWN) {
                //         Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                //     }
                //     if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTShute) == ButtonStatus.DOWN) {
                //         Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                //     }
                //     if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeD) == ButtonStatus.DOWN) {
                //         Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                //     }
                //     if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeU) == ButtonStatus.DOWN) {
                //         Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                //         break;
                //     }
                //     break;
                // }
        //     }
        //     // }
        // } else { // ARM_ELEVATOR MANUAL == true 
        //     switch (robotSettings.DRIVE_STYLE) {
        //         case MIDI: {
        //             if (midiTop.get(ControllerEnums.MidiController.R2C5) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //                 Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
        //             }
        //             if (midiTop.get(ControllerEnums.MidiController.R2C6) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //                 Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
        //             }
        //             break;
        //         }
                //  case STANDARD_2023: {
                    
                //       if (panel2.get(ControllerEnums.MidiController.R2C5) ==
                //       DefaultControllerEnums.ButtonStatus.DOWN) {
                //       Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                //       }
                //       if (panel2.get(ControllerEnums.MidiController.R2C6) ==
                //       DefaultControllerEnums.ButtonStatus.DOWN) {
                //       Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                //       break;
                //       }
                     
                // }
        //     }
        // }

    //}