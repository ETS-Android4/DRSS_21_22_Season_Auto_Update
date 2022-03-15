package org.firstinspires.ftc.teamcode.subsystems.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;

/**
 * Created by Antoine on 2/3/2022
 */
public class BluePARKTrajectory{

    CompMecanumDrive drive;

    public Trajectory initialMoveTrajectory;
    public Trajectory park;

    public enum TrajectoryControlState {
        IDLE,
        INITIAL_LIFT,
        INITIAL_MOVE,
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

    public BluePARKTrajectory(CompMecanumDrive drive, Telemetry telemetry) {
        this.drive = drive;

        trajectoryControlState = TrajectoryControlState.IDLE;

        // Define our start pose
        Pose2d startPose = new Pose2d(8.875, 62.5, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);

        telemetry.addData("Creating: ", "Trajectories");
        telemetry.update();

        initialMoveTrajectory = drive.trajectoryBuilder(startPose)
                .forward(34)
                .build();

        park = drive.trajectoryBuilder(initialMoveTrajectory.end())
                .strafeRight(0.5)
                .build();

        telemetry.addData("Trajectories: ", "Created");
        telemetry.update();

    }
}
