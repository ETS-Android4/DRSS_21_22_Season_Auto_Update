package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

	double speedOverride = 1;

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
					speedOverride = 1;
					if (controls.speedTrigger.isDown()) {
						robot.states.speedState = States.SpeedState.THREE_QUARTER_SPEED;
					}
					if (controls.quarterSpeedButton.isDown()) {
						robot.states.speedState = States.SpeedState.QUARTER_SPEED;
					}

				case THREE_QUARTER_SPEED:
					speedOverride = 0.75;
					if (!controls.speedTrigger.isDown()) {
						robot.states.speedState = States.SpeedState.FULL_SPEED;
					}

				case QUARTER_SPEED:
					speedOverride = 0.25;
					if (controls.fullSpeedButton.isDown()) {
						robot.states.speedState = States.SpeedState.FULL_SPEED;
					}

				default:
					robot.states.speedState = States.SpeedState.FULL_SPEED;
			}

			/*Drivetrain Control*/
			robot.drive.setWeightedDrivePower(
					new Pose2d(
							-gamepad1.left_stick_y * speedOverride,
							-gamepad1.left_stick_x * speedOverride,
							-gamepad1.right_stick_x * speedOverride
					)
			);
			robot.drive.update();
			Pose2d poseEstimate = robot.drive.getPoseEstimate();

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
