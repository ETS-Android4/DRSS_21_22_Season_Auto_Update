package org.firstinspires.ftc.teamcode.subsystems.states;

/**
 * Created by Antoine on 11/21/2021
 */
public class States {

    public enum SpeedState {
        FULL_SPEED,
        THREE_QUARTER_SPEED,
        HALF_SPEED,
        QUARTER_SPEED
    };
    public SpeedState speedState = SpeedState.FULL_SPEED;

}
