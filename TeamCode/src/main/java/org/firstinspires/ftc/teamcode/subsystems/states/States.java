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
    public DriveDirectionState driveDirectionState = DriveDirectionState.FORWARD;

    public enum SpeedState {
        FULL_SPEED,
        THREE_QUARTER_SPEED,
        HALF_SPEED,
        QUARTER_SPEED
    };
    public SpeedState speedState = SpeedState.FULL_SPEED;

    public enum IntakeState {
        IDLE,
        INTAKE,
        OUTTAKE
    }
    public IntakeState intakeState = IntakeState.IDLE;

    public enum GantryState {
        IDLE,
        FORWARD,
        REVERSE,
        POSTITION_CONTROL
    }
    public GantryState gantryState = GantryState.IDLE;

    public enum PusherState {
        RETRACTED,
        EXTENDED
    }
    public PusherState pusherState = PusherState.RETRACTED;

}
