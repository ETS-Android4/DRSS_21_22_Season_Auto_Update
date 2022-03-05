package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.subsystems.states.States;
import org.firstinspires.ftc.teamcode.subsystems.trajectories.BlueDepotRemoteTrajectory;
import org.firstinspires.ftc.teamcode.subsystems.trajectories.CAROUSELRedTrajectory;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebcamRed;

/**
 * Created by Antoine on 2/3/2022
 */

@Autonomous(name = "CAROUSEL Red Depot", group = "Autonomous")
public class CAROUSELRedAuto extends LinearOpMode {

    CompRobot robot;
    CAROUSELRedTrajectory trajectory;
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
        trajectory = new CAROUSELRedTrajectory(robot.drive, telemetry);
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
        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.INITIAL_LIFT;
        trajectory.placeControlState = CAROUSELRedTrajectory.PlaceControlState.SET_HEIGHT;

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
                            trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.INITIAL_MOVE;
                            break;
                    }
                    break;

                case INITIAL_MOVE:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(trajectory.carouselAlignmentTrajectory);
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.CAROUSEL_ALIGNMENT;
                    }
                    break;

                case CAROUSEL_ALIGNMENT:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(trajectory.pushIntoCarouselTrajectory);
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.PUSH_INTO_CAROUSEL;
                    }
                    break;

                case PUSH_INTO_CAROUSEL:
                    if (!robot.drive.isBusy()) {
                        generalTimer.reset();
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.SPIN_DUCK;
                    }
                    break;

                case SPIN_DUCK:
                    while (generalTimer.seconds() < 2) {
                        robot.spinner.runSpinner(0.7);
                    }
                    if (generalTimer.seconds() >= 2.5) {
                        robot.spinner.stop();

                        robot.drive.followTrajectoryAsync(trajectory.randomizedPlaceTrajectory);
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.RANDOMIZED_PLACE_TRAJECTORY;
                    }

                    break;

                case RANDOMIZED_PLACE_TRAJECTORY:
                    if (!robot.drive.isBusy()) {
                        robot.states.gantryState = States.GantryState.EXTENDING;
                        trajectory.placeControlState = CAROUSELRedTrajectory.PlaceControlState.WAIT_FOR_HEIGHT;
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.RANDOMIZED_PLACE;
                    }
                    break;

                case RANDOMIZED_PLACE:
                    switch (trajectory.placeControlState) {
                        case WAIT_FOR_HEIGHT:
                            /*
                            * TODO: Figure out what the heck is wrong here??
                            * */
                            if (robot.lift.getHeight() > (liftCustomHeight-2)) {
                                trajectory.placeControlState = CAROUSELRedTrajectory.PlaceControlState.MOVE_GANTRY;
                            }
                            break;

                        case MOVE_GANTRY:
                            if (duckPosition == 1) {
                                gantryExtension = 0.35;
                            }
                            if (duckPosition == 2) {
                                gantryExtension = 0.35;
                            }
                            if (duckPosition == 3) {
                                gantryExtension = 1;
                            }
                            if (robot.gantry.getPosition() <= (robot.gantry.DRIVER_POSTION_MIN + (robot.gantry.DRIVER_POSITON_RANGE * gantryExtension) - 1)) {
                                generalTimer.reset();
                                trajectory.placeControlState = CAROUSELRedTrajectory.PlaceControlState.PLACE;
                            }
                            break;

                        case PLACE:
                            robot.states.pusherState = States.PusherState.EXTENDED;
                            if (generalTimer.seconds() > 0.3) {
                                robot.states.pusherState = States.PusherState.RETRACTED;
                            }

                            if (generalTimer.seconds() > 0.75) {
                                generalTimer.reset();
                                trajectory.placeControlState = CAROUSELRedTrajectory.PlaceControlState.RESET;
                            }
                            break;

                        case RESET:
                            gantryExtension = 0;
                            duckPosition = 3;
                            if (generalTimer.seconds() > 1) {
                                robot.states.liftControlState = States.LiftControlState.HOME;
                                /*
                                * TODO: Again, figure out what was up with this??
                                * */
                                if (robot.lift.getHeight() <= 2.0) {
                                    robot.drive.followTrajectoryAsync(trajectory.depotAlignmentTrajectory1);
                                    trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.DEPOT_ALIGNMENT_TRAJECTORY1;
                                }
                            }
                            break;
                    }
                    break;

                case DEPOT_ALIGNMENT_TRAJECTORY1:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(trajectory.depotAlignmentTrajectory2);
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.DEPOT_ALIGNMENT_TRAJECTORY2;
                    }
                    break;

                case DEPOT_ALIGNMENT_TRAJECTORY2:
                    if (!robot.drive.isBusy()) {
                        robot.drive.setPoseEstimate(new Pose2d(10, -65, Math.toRadians(0)));

                        robot.states.gantryState = States.GantryState.DOCK;

                        robot.drive.followTrajectoryAsync(trajectory.depotAlignmentTrajectory3);
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.DEPOT_ALIGNMENT_TRAJECTORY3;
                    }
                    break;

                case DEPOT_ALIGNMENT_TRAJECTORY3:
                    if (!robot.drive.isBusy()) {
                        robot.drive.followTrajectoryAsync(trajectory.initialDepotTrajectory);
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.INITIAL_DEPOT_TRAJECTORY;
                    }
                    break;

                case INITIAL_DEPOT_TRAJECTORY:
                    if (!robot.drive.isBusy()) {
                        robot.drive.setWeightedDrivePower(
                                new Pose2d(
                                        0.25,
                                        0,
                                        0
                                )
                        );

                        robot.states.intakeState = States.IntakeState.INTAKE;
                        generalTimer.reset();
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.INTAKE;
                    }
                    break;

                case INTAKE:
                    if ((robot.intake.isLoaded()) || (generalTimer.seconds() > 2)){
                        robot.drive.setWeightedDrivePower(
                                new Pose2d(
                                        0,
                                        0,
                                        0
                                )
                        );
                        generalTimer.reset();
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.PARK;
                    }
                    break;

                case PARK:
                    if (generalTimer.seconds() > 2) {
                        trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.IDLE;
                    }
                    break;

                case IDLE:
                    requestOpModeStop();
                    break;

                default:
                    trajectory.trajectoryControlState = CAROUSELRedTrajectory.TrajectoryControlState.IDLE;
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
