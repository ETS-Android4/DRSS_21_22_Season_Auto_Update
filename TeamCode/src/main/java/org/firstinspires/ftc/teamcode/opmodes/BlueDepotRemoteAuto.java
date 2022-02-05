package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.subsystems.states.States;
import org.firstinspires.ftc.teamcode.subsystems.trajectories.BlueDepotRemoteTrajectory;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;

/**
 * Created by Antoine on 2/3/2022
 */

@Autonomous(name = "Blue Depot Remote", group = "Autonomous")
public class BlueDepotRemoteAuto extends LinearOpMode {

    CompRobot robot;
    BlueDepotRemoteTrajectory trajectory;
    Webcam webcam;

    int duckPosition = 1;
    double gantryExtension = 0;
    double liftCustomHeight = 0;
    double capstonePosition = 0;
    double cycleTime = 0;

    Pose2d poseEstimate;

    ElapsedTime matchTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new CompRobot(hardwareMap, telemetry, true);
        trajectory = new BlueDepotRemoteTrajectory(robot.drive, telemetry);
        webcam = new Webcam(hardwareMap, telemetry);

        /*Pre-Start/Post-Init Loop*/
        while (!opModeIsActive()) {
            duckPosition = webcam.locateDuck();
            telemetry.addData("Position: ", duckPosition);
            packet.put("Position: ", duckPosition);

            telemetry.addData("Robot", "Initialized");
            packet.put("Robot", "Initialized");

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }

        matchTimer.reset();

        while (opModeIsActive()) {

            /*
             *
             *
             * Beginning of Loop Updates
             *
             *
             */

            robot.drive.update();
            poseEstimate = robot.drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            /*
            *
            *
            * Autonomous Path Control State Machines
            *
            *
            */

            switch (trajectory.trajectoryControlState) {
                case IDLE:
                    trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.RANDOMIZED_PLACE_TRAJECTORY;
                    robot.drive.followTrajectoryAsync(trajectory.randomizedPlaceTrajectory);
                    break;

                case RANDOMIZED_PLACE_TRAJECTORY:
                    if (!robot.drive.isBusy()) {
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.RANDOMIZED_PLACE;
                    }
                    break;

                case RANDOMIZED_PLACE:
                    trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.DUCK_SPINNER_TRAJECTORY;
                    robot.drive.followTrajectoryAsync(trajectory.duckSpinnerTrajectory);
                    break;

                case DUCK_SPINNER_TRAJECTORY:
                    if (!robot.drive.isBusy()) {
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.SPIN_DUCK;
                    }
                    break;

                case SPIN_DUCK:
                    trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.DEPOT_ALIGNMENT_TRAJECTORY1;
                    robot.drive.followTrajectoryAsync(trajectory.depotAlignmentTrajectory1);
                    break;

                case DEPOT_ALIGNMENT_TRAJECTORY1:
                    if (!robot.drive.isBusy()) {
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.DEPOT_ALIGNMENT_TRAJECTORY2;
                        robot.drive.followTrajectoryAsync(trajectory.depotAlignmentTrajectory2);
                    }
                    break;

                case DEPOT_ALIGNMENT_TRAJECTORY2:
                    if (!robot.drive.isBusy()) {
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.INITIAL_DEPOT_TRAJECTORY;
                        robot.drive.followTrajectoryAsync(trajectory.initialDepotTrajectory);
                    }
                    break;

                case INITIAL_DEPOT_TRAJECTORY:
                    if (!robot.drive.isBusy()) {
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.INTAKE;
                    }
                    break;

                case INTAKE:
                    trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.IDLE;
                    break;

                case CHECK_CYCLE:
                    double timeLeft = 30 - matchTimer.seconds();

                    if (timeLeft > cycleTime) {
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.CYCLE_DEPOT_EXIT_TRAJECTORY;
                    }
                    else {
                        Trajectory parkTrajectory = robot.drive.trajectoryBuilder(poseEstimate)
                                .splineToLinearHeading(new Pose2d(42, 45, Math.toRadians(0.0)), Math.toRadians(0.0))
                                .build();
                        robot.drive.followTrajectoryAsync(parkTrajectory);
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.PARK;
                    }

                    break;

                case PARK:
                    if (!robot.drive.isBusy()) {
                        trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.IDLE;
                    }
                    break;

                default:
                    trajectory.trajectoryControlState = BlueDepotRemoteTrajectory.TrajectoryControlState.IDLE;
                    break;
            }

            /*
            *
            *
            * Robot Systems Control State Machines
            *
            *
            */

            /*Intake Control State Machine*/
            switch(robot.states.intakeState) {
                case IDLE:
                    robot.intake.stop();
                    break;

                case INTAKE:
                    robot.intake.runIntake(1.0);
                    if (robot.intake.isLoaded()) {
                        intakeTimer.reset();
                        robot.states.intakeState = States.IntakeState.UNLOAD;
                    }
                    break;

                case OUTTAKE:
                    robot.intake.runIntake(-1.0);
                    break;

                case UNLOAD:
                    if (intakeTimer.seconds() < 1) {
                        robot.intake.runIntake(-1.0);
                    }
                    if (intakeTimer.seconds() > .5) {
                        robot.intake.runIntake(0.0);
                        if (intakeTimer.seconds() > 2.5) {
                            robot.states.intakeState = States.IntakeState.IDLE;
                        }
                    }
                    break;

                default:
                    robot.states.intakeState = States.IntakeState.IDLE;
                    break;
            }

            /*Gantry Control State Machine*/
            switch (robot.states.gantryState) {
                case IDLE:
                    robot.gantry.stop();
                    break;

                case DOCK:
                    robot.gantry.kP = -0.025;
                    robot.gantry.update(robot.gantry.DOCK_POSTION);
                    break;

                case POSITION_CONTROL:
                    robot.gantry.kP = -0.02;
                    double CalculatedPosition = robot.gantry.DRIVER_POSTION_MIN + Range.clip(
                            (robot.gantry.DRIVER_POSITON_RANGE * gantryExtension),
                            robot.gantry.DRIVER_POSITON_RANGE,
                            0
                    );
                    robot.gantry.update(CalculatedPosition);
                    break;

                case EXTENDING:
                    robot.gantry.kP = -0.025;
                    robot.gantry.update(robot.gantry.DRIVER_POSTION_MIN);

                    if (robot.gantry.getPosition() <= robot.gantry.DRIVER_POSTION_MIN) {
                        robot.states.liftControlState = robot.states.desiredliftControlState;
                        robot.states.gantryState = States.GantryState.POSITION_CONTROL;
                    }
                    break;

                case RETRACTING:
                    robot.gantry.kP = -0.025;
                    robot.states.liftControlState = States.LiftControlState.HOME;
                    if (robot.lift.getHeight() < 1.0) {
                        robot.states.gantryState = States.GantryState.DOCK;
                    }
                    break;

                default:
                    robot.states.gantryState = States.GantryState.IDLE;
                    break;
            }

            /*Pusher Control State Machine*/
            switch (robot.states.pusherState) {
                case EXTENDED:
                    robot.pusher.pusherSetPosition(180);
                    break;

                case RETRACTED:
                    robot.pusher.pusherSetPosition(0);
                    break;

                default:
                    robot.states.pusherState = States.PusherState.RETRACTED;
                    break;
            }

            /*Lift Height State Machine*/
            switch (robot.states.liftControlState) {
                case HOME:
                    robot.lift.setHeight(0);
                    break;

                case LEVEL_ONE:
                    robot.lift.setHeight(5);
                    break;

                case LEVEL_TWO:
                    robot.lift.setHeight(10);
                    break;

                case LEVEL_THREE:
                    robot.lift.setHeight(15);
                    break;

                case CAPSTONE:
                    robot.lift.setHeight(13);
                    break;

                case HOLD:
                    robot.lift.setHeight(liftCustomHeight);
                    break;

                default:
                    robot.states.liftControlState = States.LiftControlState.HOME;
                    break;
            }

            /*Lift Control State Machine */
            switch (robot.states.liftState) {
                case IDLE:
                    robot.lift.stop();
                    break;

                case POSITION_CONTROL:
                    robot.lift.update();
                    break;

                default:
                    robot.states.liftState = States.LiftState.IDLE;
                    break;
            }

            /*Capstone Control State Machine */
            switch (robot.states.capstoneControlState) {
                case UP:
                    robot.capstone.capSetPosition(0);
                    break;

                case DOWN:
                    robot.capstone.capSetPosition(180);
                    break;

                case POSITION_CONTROL:
                    robot.capstone.capSetPosition(capstonePosition);
                    break;

                default:
                    robot.states.capstoneControlState = States.CapstoneControlState.POSITION_CONTROL;
                    break;
            }

            /*
            *
            *
            * Telemetry
            *
            *
            */

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);

        }

    }
}
