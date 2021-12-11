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

				case THREE_QUARTER_SPEED:
					speedOverride = 0.75;
					if (!controls.speedTrigger.isDown()) {
						robot.states.speedState = States.SpeedState.FULL_SPEED;
					}

				case HALF_SPEED:
					speedOverride = 0.5;
					if (controls.fullSpeedButton.isDown()) {
						robot.states.speedState = States.SpeedState.FULL_SPEED;
					}

				default:
					robot.states.speedState = States.SpeedState.FULL_SPEED;
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

				case REVERSE:
					robot.drive.setWeightedDrivePower(
							new Pose2d(
									gamepad1.left_stick_y * speedOverride,
									gamepad1.left_stick_x * speedOverride,
									gamepad1.right_stick_x * speedOverride
							)
					);

					if (controls.driveFlipButton.wasJustPressed()) {
						robot.states.driveDirectionState = States.DriveDirectionState.FORWARD;
					}

				default:
					robot.states.driveDirectionState = States.DriveDirectionState.FORWARD;
			}
			robot.drive.update();
			Pose2d poseEstimate = robot.drive.getPoseEstimate();

			/*Intake Control*/
			switch(robot.states.intakeState) {
				case IDLE:
					robot.intake.stop();
					if (controls.intakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.INTAKE;
					}
					if (controls.outtakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.OUTTAKE;
					}

				case INTAKE:
					robot.intake.runIntake(1.0);
					if (!controls.intakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.IDLE;
					}

				case OUTTAKE:
					robot.intake.runIntake(-1.0);
					if (!controls.outtakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.IDLE;
					}

				default:
					robot.states.intakeState = States.IntakeState.IDLE;
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
