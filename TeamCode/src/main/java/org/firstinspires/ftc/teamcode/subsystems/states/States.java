package org.firstinspires.ftc.teamcode.subsystems.states;

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
        OUTTAKE
    }
    public IntakeState intakeState;

    public enum GantryState {
        IDLE,
        FORWARD,
        REVERSE,
        DOCK,
        DRIVER_POSITION,
        POSITION_CONTROL
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
        CAPSTONE
    }
    public LiftControlState liftControlState;

    public States() {
        driveDirectionState = DriveDirectionState.FORWARD;
        speedState = SpeedState.FULL_SPEED;
        intakeState = IntakeState.IDLE;
        gantryState = GantryState.IDLE;
        pusherState = PusherState.RETRACTED;
        liftState = LiftState.IDLE;
        liftControlState = LiftControlState.LEVEL_TWO;
    }

}
