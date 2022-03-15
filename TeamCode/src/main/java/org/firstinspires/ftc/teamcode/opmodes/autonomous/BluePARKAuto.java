package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.subsystems.states.States;
import org.firstinspires.ftc.teamcode.subsystems.trajectories.BluePARKTrajectory;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebcamRed;

/**
 * Created by Antoine on 2/3/2022
 */

@Autonomous(name = "Blue Park", group = "Autonomous")
public class BluePARKAuto extends LinearOpMode {

    CompRobot robot;
    BluePARKTrajectory trajectory;
    WebcamRed webcam;

    int duckPosition = 1;
    double gantryExtension = 0;
    double liftCustomHeight = 0;
    double capstonePosition = 0;
    double cycleTime = 10;

    Pose2d poseEstimate;

    ElapsedTime matchTimer = new ElapsedTime();
    ElapsedTime generalTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime pusherTimer = new ElapsedTime();

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new CompRobot(hardwareMap, telemetry, true);
        trajectory = new BluePARKTrajectory(robot.drive, telemetry);
        webcam = new WebcamRed(hardwareMap, telemetry);

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
        trajectory.trajectoryControlState = BluePARKTrajectory.TrajectoryControlState.INITIAL_LIFT;
        trajectory.placeControlState = BluePARKTrajectory.PlaceControlState.SET_HEIGHT;

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
                case INITIAL_LIFT:
                    switch (trajectory.placeControlState) {
                        case SET_HEIGHT:
                            robot.gantry.DRIVER_POSTION_MIN = -135;
                            if (duckPosition == 1) {
                                liftCustomHeight = 5;
                            }
                            if (duckPosition == 2) {
                                liftCustomHeight = 10;
                            }
                            if (duckPosition == 3) {
                                liftCustomHeight = 14.5;
                            }
                            robot.drive.followTrajectoryAsync(trajectory.initialMoveTrajectory);
                            trajectory.trajectoryControlState = BluePARKTrajectory.TrajectoryControlState.INITIAL_MOVE;
                            break;
                    }
                    break;
                    
                case INITIAL_MOVE:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(trajectory.park);
                        trajectory.trajectoryControlState = BluePARKTrajectory.TrajectoryControlState.PARK;
                    }
                    break;
                    
                case PARK:
                    if (!robot.drive.isBusy()) {
                        trajectory.trajectoryControlState = BluePARKTrajectory.TrajectoryControlState.IDLE;
                    }
                    break;

                case IDLE:
                    requestOpModeStop();
                    break;

                default:
                    trajectory.trajectoryControlState = BluePARKTrajectory.TrajectoryControlState.IDLE;
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
                    if (intakeTimer.seconds() > 1.5) {
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
                    robot.gantry.kP = -0.03;
                    robot.gantry.update(robot.gantry.DOCK_POSTION);
                    break;

                case POSITION_CONTROL:
                    robot.gantry.kP = -0.03;
                    double CalculatedPosition = robot.gantry.DRIVER_POSTION_MIN + Range.clip(
                            (robot.gantry.DRIVER_POSITON_RANGE * gantryExtension),
                            robot.gantry.DRIVER_POSITON_RANGE,
                            0
                    );
                    robot.gantry.update(CalculatedPosition);
                    break;

                case EXTENDING:
                    robot.gantry.kP = -0.03;
                    robot.gantry.update(robot.gantry.DRIVER_POSTION_MIN);

                    if (robot.gantry.getPosition() <= robot.gantry.DRIVER_POSTION_MIN) {
                        robot.states.liftControlState = States.LiftControlState.HOLD;
                        robot.states.gantryState = States.GantryState.POSITION_CONTROL;
                    }
                    break;

                case RETRACTING:
                    robot.gantry.kP = -0.03;
                    robot.states.liftControlState = States.LiftControlState.HOME;
                    if (robot.lift.getHeight() < 1.0) {
                        //robot.states.gantryState = States.GantryState.DOCK;
                    }
                    break;

                default:
                    robot.states.gantryState = States.GantryState.IDLE;
                    break;
            }

            /*Pusher Control State Machine*/
            switch (robot.states.pusherState) {
                case RETRACTED:
                    robot.pusher.pusherSetPosition(0.0);
                    break;

                case EXTENDED:
                    robot.pusher.pusherSetPosition(1.0);
                    if (pusherTimer.seconds() > 0.3) {
                        robot.states.pusherState = States.PusherState.RETRACTED;
                    }
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
            telemetry.addLine("Position")
                    .addData("x", poseEstimate.getX())
                    .addData("y", poseEstimate.getY())
                    .addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            telemetry.addData("Height", robot.lift.getHeight());

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);

        }

    }
}
