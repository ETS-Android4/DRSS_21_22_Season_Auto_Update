package org.firstinspires.ftc.teamcode.subsystems.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;

/**
 * Created by Antoine on 2/3/2022
 */
public class CAROUSELRedTrajectory{

    CompMecanumDrive drive;

    public Trajectory initialMoveTrajectory;
    public Trajectory carouselAlignmentTrajectory;
    public Trajectory pushIntoCarouselTrajectory;
    public Trajectory randomizedPlaceTrajectory;
    public Trajectory depotAlignmentTrajectory1;
    public Trajectory depotAlignmentTrajectory2;
    public Trajectory depotAlignmentTrajectory3;
    public Trajectory initialDepotTrajectory;

    public enum TrajectoryControlState {
        IDLE,
        INITIAL_LIFT,
        INITIAL_MOVE,
        CAROUSEL_ALIGNMENT,
        PUSH_INTO_CAROUSEL,
        SPIN_DUCK,
        RANDOMIZED_PLACE_TRAJECTORY,
        RANDOMIZED_PLACE,
        DEPOT_ALIGNMENT_TRAJECTORY1,
        DEPOT_ALIGNMENT_TRAJECTORY2,
        DEPOT_ALIGNMENT_TRAJECTORY3,
        INITIAL_DEPOT_TRAJECTORY,
        INTAKE,
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

    public CAROUSELRedTrajectory(CompMecanumDrive drive, Telemetry telemetry) {
        this.drive = drive;

        trajectoryControlState = TrajectoryControlState.IDLE;

        // Define our start pose
        Pose2d startPose = new Pose2d(-33.625, -62.5, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);

        telemetry.addData("Creating: ", "Trajectories");
        telemetry.update();

        initialMoveTrajectory = drive.trajectoryBuilder(startPose)
                .strafeLeft(10)
                .build();

        carouselAlignmentTrajectory = drive.trajectoryBuilder(initialMoveTrajectory.end())
                .lineToLinearHeading(new Pose2d(-58, -52.5, Math.toRadians(180.0)))
                .build();

        pushIntoCarouselTrajectory = drive.trajectoryBuilder(carouselAlignmentTrajectory.end())
                .strafeLeft(4)
                .build();

        randomizedPlaceTrajectory = drive.trajectoryBuilder(carouselAlignmentTrajectory.end())
                .lineToLinearHeading(new Pose2d(-26, -38, Math.toRadians(240)))
                .build();

        depotAlignmentTrajectory1 = drive.trajectoryBuilder(randomizedPlaceTrajectory.end())
                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(0.0)))
                .build();

        depotAlignmentTrajectory2 = drive.trajectoryBuilder(depotAlignmentTrajectory1.end())
                .strafeRight(7)
                .build();

        depotAlignmentTrajectory3 = drive.trajectoryBuilder(new Pose2d(10, -65, Math.toRadians(0)))
                .strafeLeft(1)
                .build();

        initialDepotTrajectory = drive.trajectoryBuilder(depotAlignmentTrajectory3.end())
                .forward(27)
                .build();

        telemetry.addData("Trajectories: ", "Created");
        telemetry.update();

    }
}
