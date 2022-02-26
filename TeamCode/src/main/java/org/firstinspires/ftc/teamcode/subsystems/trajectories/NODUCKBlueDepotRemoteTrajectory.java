package org.firstinspires.ftc.teamcode.subsystems.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;

/**
 * Created by Antoine on 2/3/2022
 */
public class NODUCKBlueDepotRemoteTrajectory{

    CompMecanumDrive drive;

    public Trajectory randomizedPlaceTrajectory;
    public Trajectory depotAlignmentTrajectory1;
    public Trajectory depotAlignmentTrajectory2;
    public Trajectory depotAlignmentTrajectory3;
    public Trajectory initialDepotTrajectory;
    public Trajectory depotExitTrajectory1;
    public Trajectory depotExitTrajectory2;
    public Trajectory depotExitTrajectory3;
    public Trajectory cycleAlignToPlaceTrajectory;
    public Trajectory cycleAlignToDepotTrajectory;
    public Trajectory cycleEnterDepotTrajectory;
    public Trajectory park;

    public enum TrajectoryControlState {
        IDLE,
        INITIAL_LIFT,
        RANDOMIZED_PLACE_TRAJECTORY,
        RANDOMIZED_PLACE,
        DEPOT_ALIGNMENT_TRAJECTORY1,
        DEPOT_ALIGNMENT_TRAJECTORY2,
        DEPOT_ALIGNMENT_TRAJECTORY3,
        INITIAL_DEPOT_TRAJECTORY,
        INTAKE,
        CHECK_CYCLE,
        CYCLE_DEPOT_EXIT_TRAJECTORY1,
        CYCLE_DEPOT_EXIT_TRAJECTORY2,
        CYCLE_DEPOT_EXIT_TRAJECTORY3,
        CYCLE_ALIGN_TO_PLACE_TRAJECTORY,
        CYCLE_PLACE,
        CYCLE_ALIGN_TO_DEPOT_TRAJECTORY,
        PARK
    }
    public TrajectoryControlState trajectoryControlState;

    public enum PlaceControlState {
        SET_HEIGHT,
        WAIT_FOR_HEIGHT,
        MOVE_GANTRY,
        PLACE,
        RESET
    }
    public PlaceControlState placeControlState;

    public NODUCKBlueDepotRemoteTrajectory(CompMecanumDrive drive, Telemetry telemetry) {
        this.drive = drive;

        trajectoryControlState = TrajectoryControlState.IDLE;

        // Define our start pose
        Pose2d startPose = new Pose2d(8.875, 62.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        telemetry.addData("Creating: ", "Trajectories");
        telemetry.update();

        randomizedPlaceTrajectory = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-20, 40, Math.toRadians(120)))
                .build();

        depotAlignmentTrajectory1 = drive.trajectoryBuilder(randomizedPlaceTrajectory.end())
                .lineToLinearHeading(new Pose2d(10, 60, Math.toRadians(0.0)))
                .build();

        depotAlignmentTrajectory2 = drive.trajectoryBuilder(depotAlignmentTrajectory1.end())
                .strafeLeft(7)
                .build();

        depotAlignmentTrajectory3 = drive.trajectoryBuilder(new Pose2d(10, 65, Math.toRadians(0)))
                .strafeRight(2)
                .build();

        /*
        * TODO: Adjust this so that the weighted drive power doesn't have to go forward too much before hitting the pile.
        * */
        initialDepotTrajectory = drive.trajectoryBuilder(depotAlignmentTrajectory3.end())
                .forward(27)
                .build();

        telemetry.addData("Trajectories: ", "Created");
        telemetry.update();

    }
}
