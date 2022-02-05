package org.firstinspires.ftc.teamcode.subsystems.trajectories;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;

/**
 * Created by Antoine on 2/3/2022
 */
public class BlueDepotRemoteTrajectory{

    CompMecanumDrive drive;

    enum TrajectoryControlState {
        IDLE,
        RANDOMIZED_PLACE_TRAJECTORY,
        RANDOMIZED_PLACE,
        DUCK_SPINNER_TRAJECTORY,
        SPIN_DUCK,
        DEPOT_ALIGNMENT_TRAJECTORY,
        INITIAL_DEPOT_TRAJECTORY,
        INTAKE,
        CYCLE_DEPOT_EXIT_TRAJECTORY,
        CYCLE_ALIGN_TO_PLACE_TRAJECTORY,
        PLACE,
        CYCLE_ALIGN_TO_DEPOT_TRAJECTORY,
        CYCLE_DEPOT_TRAJECTORY,
        PARK_DEPOT_ENTRANCE_TRAJECTORY,
        PARK
    }
    TrajectoryControlState trajectoryControlState;

    public BlueDepotRemoteTrajectory(CompMecanumDrive drive) {
        this.drive = drive;

        trajectoryControlState = TrajectoryControlState.IDLE;
    }
}
