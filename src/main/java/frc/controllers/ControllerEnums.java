package frc.controllers;

import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.ControllerInterfaces;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.controllers.basecontrollers.XBoxController;

import java.util.function.Function;

import static frc.controllers.basecontrollers.BaseController.IValidController;

/**
 * These enums are used for each controller. If you make a new controller, create a new enum for its mappings here for
 * safekeeping and then implement a get function in {@link BaseController} and then create a new class extending
 * BaseController that overrides that get (the gets in BaseController should all throw exceptions so if an {@link
 * XBoxController xbox controller} is queried for a {@link DefaultControllerEnums.WiiAxis wii axis} it should throw a fit)
 *
 * @see BaseController
 */
public class ControllerEnums {
    public enum CustomControllers implements IValidController {
        BUTTON_PANEL_CONTROLLER(ButtonPanelController::new);

        private final Function<Integer, BaseController> constructor;

        CustomControllers(Function<Integer, BaseController> structor) {
            constructor = structor;
        }

        public Function<Integer, BaseController> getConstructor() {
            return constructor;
        }
    }

    /**
     * @see ButtonPanelController
     */
    public enum ButtonPanelButtons implements ControllerInterfaces.IDiscreteInput {
        RAISE_CLIMBER(1), LOWER_CLIMBER(2), CLIMBER_LOCK(3), CLIMBER_UNLOCK(4), BUDDY_CLIMB(5), AUX_TOP(6), AUX_BOTTOM(7), INTAKE_UP(8), INTAKE_DOWN(9), HOPPER_IN(10), HOPPER_OUT(11), TARGET(12), SOLID_SPEED(13);

        public final int AXIS_VALUE;

        ButtonPanelButtons(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    /**
     * @see ButtonPanelController
     */
    public enum ButtonPanelTapedButtons implements ControllerInterfaces.IDiscreteInput {
        RAISE_CLIMBER(1), LOWER_CLIMBER(2), CLIMBER_LOCK(3), CLIMBER_UNLOCK(4), BUDDY_CLIMB(5), AUX_TOP(6), HOOD_POS_1(7), SINGLE_SHOT(8), HOOD_POS_2(9), SOLID_SPEED(10), HOOD_POS_3(11), TARGET(12), HOOD_POS_4(13);

        public final int AXIS_VALUE;

        ButtonPanelTapedButtons(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    public enum ButtonPanelButtons2022 implements ControllerInterfaces.IDiscreteInput {
        LOW_SHOT(1), INTAKE_UP(6), INTAKE_DOWN(11), FENDER_SHOT(2), TARMAC_SHOT(7), FAR_SHOT(12), PIVOT_PISTON_UP(10), PIVOT_PISTON_DOWN(9), AUX_3(13), AUX_2(14), AUX_1(15), AUX_4(8), AUX_5(3), FIRST_STAGE_DOWN(4), FIRST_STAGE_UP(5);

        public final int AXIS_VALUE;

        ButtonPanelButtons2022(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    public enum TCannonExtraButtons implements ControllerInterfaces.IDiscreteInput {
        TILT_TOP(5), TILT_MID(4), TILT_LOW(3),  FIRE(8);

        public final int AXIS_VALUE;

        TCannonExtraButtons(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }
    //Row # colum # is writen as R#C#
    public enum MidiController implements ControllerInterfaces.IDiscreteInput {
        R1C1(1), R1C2(2), R1C3(3),  R1C4(4), R1C5(5), R1C6(6), R1C7(7),  R1C8(8),
        R2C1(9), R2C2(10), R2C3(11),  R2C4(12), R2C5(13), R2C6(14), R2C7(15),  R2C8(16),
        R3C1(17), R3C2(18), R3C3(19),  R3C4(20), R3C5(21), R3C6(22), R3C7(23),  R3C8(24),
        R4C1(25), R4C2(26), R4C3(27),  R4C4(28), R4C5(29), R4C6(30), R4C7(31),  R4C8(32);
    
        public final int AXIS_VALUE;

        MidiController(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }
    public enum ButtonPanelButtonsPlacement2023 implements ControllerInterfaces.IDiscreteInput{
        //left to right and u can relable them later
        T1(10), T2(12), T3(11), T4(7), T5(8), T6(9),
        T7(3), T8(4), T9(5), SpikePickU(13), Stable(2);  
        
        public final int AXIS_VALUE;

        ButtonPanelButtonsPlacement2023(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }

    //needs to be in 
    public enum ButtonPanelButtonsElse2023 implements ControllerInterfaces.IDiscreteInput{
        Cone(11), Cube(12), Climb(7), GTStation1(10), GTStation2(13), 
        SpikeD(8), SpikeU(9), GTShute(2), High(4), 
        Mid(6), Low(5), Floor(3);

        public final int AXIS_VALUE;

        ButtonPanelButtonsElse2023(int value) {
            this.AXIS_VALUE = value;
        }

        @Override
        public int getChannel() {
            return AXIS_VALUE;
        }
    }
}