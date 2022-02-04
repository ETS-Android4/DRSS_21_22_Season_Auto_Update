package org.firstinspires.ftc.teamcode.subsystems.states;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.gantry.Gantry;
import org.firstinspires.ftc.teamcode.subsystems.pusher.Pusher;

/**
 * Created by Antoine on 11/21/2021
 */

public class States {

    public enum DriveDirectionState {
        FORWARD,
        REVERSE
    }
    public DriveDirectionState driveDirectionState;

    public enum SpeedState {
        FULL_SPEED,
        THREE_QUARTER_SPEED,
        HALF_SPEED,
        QUARTER_SPEED
    };
    public SpeedState speedState;

    public enum IntakeState {
        IDLE,
        INTAKE,
        OUTTAKE,
        UNLOAD
    }
    public IntakeState intakeState;

    public enum LightState {
        OFF,
        GREEN,
        RED,
        BLINK_GREEN,
        BLINK_RED
    }
    public LightState lightState;

    public enum GantryState {
        IDLE,
        DOCK,
        POSITION_CONTROL,
        EXTENDING,
        RETRACTING,
        REHOMING
    }
    public GantryState gantryState;

    public enum PusherState {
        RETRACTED,
        EXTENDED
    }
    public PusherState pusherState;

    public enum LiftState {
        IDLE,
        EXTEND,
        RETRACT,
        POSITION_CONTROL
    }
    public LiftState liftState;

    public enum LiftControlState {
        HOME,
        LEVEL_ONE,
        LEVEL_TWO,
        LEVEL_THREE,
        CAPSTONE,
        HOLD
    }
    public LiftControlState liftControlState;
    public LiftControlState previousliftControlState;
    public LiftControlState desiredliftControlState;

    public enum CapstoneControlState {
        UP,
        DOWN,
        POSITION_CONTROL
    }
    public CapstoneControlState capstoneControlState;

    public States() {
        driveDirectionState = DriveDirectionState.FORWARD;
        speedState = SpeedState.FULL_SPEED;
        intakeState = IntakeState.IDLE;
        lightState = LightState.OFF;
        gantryState = GantryState.DOCK;
        pusherState = PusherState.RETRACTED;
        liftState = LiftState.POSITION_CONTROL;
        liftControlState = LiftControlState.HOME;
        previousliftControlState = LiftControlState.LEVEL_THREE;
        desiredliftControlState = LiftControlState.HOME;
        capstoneControlState = CapstoneControlState.UP;
    }

}
