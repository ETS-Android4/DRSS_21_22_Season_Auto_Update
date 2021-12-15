package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.controls.Controls;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.subsystems.states.States;

@TeleOp(name = "Comp TeleOp", group = "Comp")
public class CompTeleOp extends LinearOpMode{

	private GamepadEx gamepad1ex;
	private GamepadEx gamepad2ex;
	Controls controls;

	CompRobot robot;

	double speedOverride = 1.0;

	@Override
	public void runOpMode() throws InterruptedException{
		gamepad1ex = new GamepadEx(gamepad1);
		gamepad2ex = new GamepadEx(gamepad2);
		controls = new Controls(gamepad1ex, gamepad2ex);

		robot = new CompRobot(hardwareMap, telemetry);

		waitForStart();

		while(!isStopRequested()) {

			/*Speed Control State Machine*/
			switch(robot.states.speedState) {
				case FULL_SPEED:
					speedOverride = 1.0;
					if (controls.speedTrigger.isDown()) {
						robot.states.speedState = States.SpeedState.THREE_QUARTER_SPEED;
					}
					if (controls.halfSpeedButton.isDown()) {
						robot.states.speedState = States.SpeedState.QUARTER_SPEED;
					}
					break;

				case THREE_QUARTER_SPEED:
					speedOverride = 0.75;
					if (!controls.speedTrigger.isDown()) {
						robot.states.speedState = States.SpeedState.FULL_SPEED;
					}
					break;

				case HALF_SPEED:
					speedOverride = 0.5;
					if (controls.fullSpeedButton.isDown()) {
						robot.states.speedState = States.SpeedState.FULL_SPEED;
					}
					break;

				default:
					robot.states.speedState = States.SpeedState.FULL_SPEED;
					break;
			}

			/*Drivetrain Control State Machine*/
			switch (robot.states.driveDirectionState) {
				case FORWARD:
					robot.drive.setWeightedDrivePower(
							new Pose2d(
									-gamepad1.left_stick_y * speedOverride,
									-gamepad1.left_stick_x * speedOverride,
									-gamepad1.right_stick_x * speedOverride
							)
					);

					if (controls.driveFlipButton.wasJustPressed()) {
						robot.states.driveDirectionState = States.DriveDirectionState.REVERSE;
					}
					break;

				case REVERSE:
					robot.drive.setWeightedDrivePower(
							new Pose2d(
									gamepad1.left_stick_y * speedOverride,
									gamepad1.left_stick_x * speedOverride,
									-gamepad1.right_stick_x * speedOverride
							)
					);

					if (controls.driveFlipButton.wasJustPressed()) {
						robot.states.driveDirectionState = States.DriveDirectionState.FORWARD;
					}
					break;

				default:
					robot.states.driveDirectionState = States.DriveDirectionState.FORWARD;
					break;
			}
			robot.drive.update();
			Pose2d poseEstimate = robot.drive.getPoseEstimate();

			/*Intake Control State Machine*/
			switch(robot.states.intakeState) {
				case IDLE:
					robot.intake.stop();
					if (controls.intakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.INTAKE;
					}
					if (controls.outtakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.OUTTAKE;
					}
					break;

				case INTAKE:
					robot.intake.runIntake(1.0);
					if (!controls.intakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.IDLE;
					}
					break;

				case OUTTAKE:
					robot.intake.runIntake(-1.0);
					if (!controls.outtakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.IDLE;
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
					if (controls.gantryForwardButton.isDown()) {
						robot.states.gantryState = States.GantryState.FORWARD;
					}
					if (controls.gantryReverseButton.isDown()) {
						robot.states.gantryState = States.GantryState.REVERSE;
					}
					break;

				case FORWARD:
					robot.gantry.setGantryPower(1);
					if (!controls.gantryForwardButton.isDown()) {
						robot.states.gantryState = States.GantryState.IDLE;
					}
					break;

				case REVERSE:
					robot.gantry.setGantryPower(-1);
					robot.states.pusherState = States.PusherState.RETRACTED;
					if (!controls.gantryReverseButton.isDown()) {
						robot.states.gantryState = States.GantryState.IDLE;
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
					if (controls.pusherRetractButton.isDown()) {
						robot.states.pusherState = States.PusherState.RETRACTED;
					}
					break;

				case RETRACTED:
					robot.pusher.pusherSetPosition(0);
					if (controls.pusherExtendButton.isDown()) {
						robot.states.pusherState = States.PusherState.EXTENDED;
					}
					break;

				default:
					robot.states.pusherState = States.PusherState.RETRACTED;
					break;
			}

			/*Lift Control State Machine */
			switch (robot.states.liftState) {
				case IDLE:
					robot.lift.stop();
					if (gamepad2ex.getLeftY() >= 0.1) {
						robot.states.liftState = States.LiftState.EXTEND;
					}
					else if (gamepad2ex.getLeftY() <= -0.1) {
						robot.states.liftState = States.LiftState.RETRACT;
					}
					break;

				case EXTEND:
					robot.lift.setLiftPower(gamepad2ex.getLeftY());
					if (gamepad2ex.getLeftY() <= 0.1) {
						robot.states.liftState = States.LiftState.IDLE;
					}
					break;

				case RETRACT:
					robot.lift.setLiftPower(gamepad2ex.getLeftY());
					if (gamepad2ex.getLeftY() >= -0.1) {
						robot.states.liftState = States.LiftState.IDLE;
					}
					break;

				default:
					robot.states.liftState = States.LiftState.IDLE;
					break;
			}

			/*Telemetry*/
			telemetry.addData("x", poseEstimate.getX());
			telemetry.addData("y", poseEstimate.getY());
			telemetry.addData("heading", poseEstimate.getHeading());
			telemetry.update();

			/*End of loop updates*/
			gamepad1ex.readButtons();
			gamepad2ex.readButtons();
			controls.readValues();
		}
	}
}
