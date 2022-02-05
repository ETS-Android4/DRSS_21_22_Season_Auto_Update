package org.firstinspires.ftc.teamcode.subsystems.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;

/**
 * Created by Antoine on 2/3/2022
 */
public class BlueDepotRemoteTrajectory{

    CompMecanumDrive drive;

    public Trajectory randomizedPlaceTrajectory;
    public Trajectory duckSpinnerTrajectory;
    public Trajectory depotAlignmentTrajectory1;
    public Trajectory depotAlignmentTrajectory2;
    public Trajectory initialDepotTrajectory;
    public Trajectory cycleAlignToPlaceTrajectory;
    public Trajectory cycleAlignToDepotTrajectory;
    public Trajectory park;

    public enum TrajectoryControlState {
        IDLE,
        RANDOMIZED_PLACE_TRAJECTORY,
        RANDOMIZED_PLACE,
        DUCK_SPINNER_TRAJECTORY,
        SPIN_DUCK,
        DEPOT_ALIGNMENT_TRAJECTORY1,
        DEPOT_ALIGNMENT_TRAJECTORY2,
        INITIAL_DEPOT_TRAJECTORY,
        INTAKE,
        CHECK_CYCLE,
        CYCLE_DEPOT_EXIT_TRAJECTORY,
        CYCLE_ALIGN_TO_PLACE_TRAJECTORY,
        CYCLE_PLACE,
        CYCLE_ALIGN_TO_DEPOT_TRAJECTORY,
        CYCLE_DEPOT_TRAJECTORY,
        PARK
    }
    public TrajectoryControlState trajectoryControlState;

    public BlueDepotRemoteTrajectory(CompMecanumDrive drive, Telemetry telemetry) {
        this.drive = drive;

        trajectoryControlState = TrajectoryControlState.IDLE;

        // Define our start pose
        Pose2d startPose = new Pose2d(8.875, 62.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        telemetry.addData("Creating: ", "Trajectories");
        telemetry.update();

        randomizedPlaceTrajectory = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-20, 45, Math.toRadians(110)))
                .build();

        duckSpinnerTrajectory = drive.trajectoryBuilder(randomizedPlaceTrajectory.end())
                .splineToLinearHeading(new Pose2d(-60, 54, Math.toRadians(90.0)), Math.toRadians(90.0))
                .build();

        depotAlignmentTrajectory1 = drive.trajectoryBuilder(duckSpinnerTrajectory.end(), true)
                .splineTo(new Vector2d(-60, 44), Math.toRadians(90.0))
                .build();

        depotAlignmentTrajectory2 = drive.trajectoryBuilder(depotAlignmentTrajectory1.end())
                .splineToLinearHeading(new Pose2d(-30, 62.5, Math.toRadians(0.0)), Math.toRadians(90.0))
                .build();

        initialDepotTrajectory = drive.trajectoryBuilder(depotAlignmentTrajectory2.end())
                .splineTo(new Vector2d(-12, 45), Math.toRadians(0.0))
                .build();

        telemetry.addData("Trajectories: ", "Created");
        telemetry.update();

    }
}
