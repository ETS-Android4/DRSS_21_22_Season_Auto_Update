package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.controls.Controls;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.subsystems.states.States;

@TeleOp(name = "Angler TeleOp", group = "Comp")
public class CompTeleOp extends LinearOpMode{

	Controls controls;

	CompRobot robot;

	double speedOverride = 1.0;

	ElapsedTime intakeTimer = new ElapsedTime();
	ElapsedTime blinkTimer = new ElapsedTime();

	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	@Override
	public void runOpMode() throws InterruptedException{
		GamepadEx gamepad1ex = new GamepadEx(gamepad1);
		GamepadEx gamepad2ex = new GamepadEx(gamepad2);
		controls = new Controls(gamepad1ex, gamepad2ex);

		robot = new CompRobot(hardwareMap, telemetry, true);

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
									gamepad1.left_stick_x * speedOverride,
									-gamepad1.right_stick_x * speedOverride
							)
					);

					if (controls.driveFlipButton.wasJustPressed()) {
						robot.states.lightState = States.LightState.GREEN;
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
						robot.states.lightState = States.LightState.RED;
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
					if (controls.outtakeButton.isDown()) {
						robot.states.intakeState = States.IntakeState.OUTTAKE;
					}
					break;

				case INTAKE:
					robot.intake.runIntake(1.0);
					if (!controls.intakeTrigger.isDown()) {
						robot.states.intakeState = States.IntakeState.IDLE;
					}
					if (robot.intake.isLoaded()) {
						if (robot.states.lightState == States.LightState.RED) {
							robot.states.lightState = States.LightState.BLINK_RED;
						}
						if (robot.states.lightState == States.LightState.GREEN) {
							robot.states.lightState = States.LightState.BLINK_GREEN;
						}
						intakeTimer.reset();
						robot.states.intakeState = States.IntakeState.UNLOAD;
					}
					break;

				case OUTTAKE:
					robot.intake.runIntake(-1.0);
					if (!controls.outtakeButton.isDown()) {
						robot.states.intakeState = States.IntakeState.IDLE;
					}
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

			/*Light Control State Machine */
			switch (robot.states.lightState) {
				case OFF:
					robot.drive.setLed(false, false);
					break;

				case RED:
					robot.drive.setLed(true, false);
					break;

				case GREEN:
					robot.drive.setLed(false, true);
					break;

				case BLINK_RED:
					blinkTimer.reset();
					for (int i = 0; i <= 9; i++) {
						if (blinkTimer.milliseconds() < 100) {
							robot.drive.setLed(true, false);
						}
						blinkTimer.reset();
						if (blinkTimer.milliseconds() < 100) {
							robot.drive.setLed(false, false);
						}
						blinkTimer.reset();
					}
					robot.states.lightState = States.LightState.RED;
					break;

				case BLINK_GREEN:
					blinkTimer.reset();
					for (int i = 0; i <= 9; i++) {
						if (blinkTimer.milliseconds() < 100) {
							robot.drive.setLed(false, true);
						}
						blinkTimer.reset();
						if (blinkTimer.milliseconds() < 100) {
							robot.drive.setLed(false, false);
						}
						blinkTimer.reset();
					}
					robot.states.lightState = States.LightState.GREEN;
					break;

				default:
					robot.states.lightState = States.LightState.OFF;
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
					if (gamepad2ex.getLeftY() <= -0.1) {
						robot.states.gantryState = States.GantryState.REHOMING;
					}
					break;

				case POSITION_CONTROL:
					robot.gantry.kP = -0.0075;
					double CalculatedPosition = robot.gantry.DRIVER_POSTION_MIN + Range.clip(
							(robot.gantry.DRIVER_POSITON_RANGE * gamepad2ex.getLeftY()),
							robot.gantry.DRIVER_POSITON_RANGE,
							0
					);
					robot.gantry.update(CalculatedPosition);
					break;

				case EXTENDING:
					robot.gantry.kP = -0.025;
					robot.gantry.update(robot.gantry.DRIVER_POSTION_MIN);

					if (robot.gantry.getPosition() <= robot.gantry.DRIVER_POSTION_MIN) {
						robot.states.liftControlState = robot.states.previousliftControlState;
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

				case REHOMING:
					if (gamepad2ex.getLeftY() <= -0.1) {
						robot.gantry.setGantryPower(gamepad2ex.getLeftY());
					}
					else {
						robot.gantry.gantryMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
						robot.gantry.gantryMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
					if (controls.pusherButton.wasJustPressed()) {
						robot.states.pusherState = States.PusherState.RETRACTED;
					}
					break;

				case RETRACTED:
					robot.pusher.pusherSetPosition(0);
					if (controls.pusherButton.wasJustPressed()) {
						robot.states.pusherState = States.PusherState.EXTENDED;
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

					if (controls.liftButton.wasJustPressed()) {
						robot.states.gantryState = States.GantryState.EXTENDING;
					}

					switch (robot.states.previousliftControlState) {
						case LEVEL_ONE:
							if (controls.liftHeightIncreaseButton.wasJustPressed()) {
								robot.states.previousliftControlState = States.LiftControlState.LEVEL_TWO;
							}
							break;

						case LEVEL_TWO:
							if (controls.liftHeightDecreaseButton.wasJustPressed()) {
								robot.states.previousliftControlState = States.LiftControlState.LEVEL_ONE;
							}
							if (controls.liftHeightIncreaseButton.wasJustPressed()) {
								robot.states.previousliftControlState = States.LiftControlState.LEVEL_THREE;
							}
							break;

						case LEVEL_THREE:
							if (controls.liftHeightDecreaseButton.wasJustPressed()) {
								robot.states.previousliftControlState = States.LiftControlState.LEVEL_TWO;
							}
							if (controls.liftHeightIncreaseButton.wasJustPressed()) {
								robot.states.previousliftControlState = States.LiftControlState.CAPSTONE;
							}
							break;

						case CAPSTONE:
							if (controls.liftHeightDecreaseButton.wasJustPressed()) {
								robot.states.previousliftControlState = States.LiftControlState.LEVEL_THREE;
							}
							break;

						default:
							robot.states.previousliftControlState = States.LiftControlState.LEVEL_ONE;
							break;
					}

					break;

				case LEVEL_ONE:
					robot.lift.setHeight(5);
					if (controls.liftHeightIncreaseButton.wasJustPressed()) {
						robot.states.previousliftControlState = States.LiftControlState.LEVEL_ONE;
						robot.states.liftControlState = States.LiftControlState.LEVEL_TWO;
					}

					if (controls.liftButton.wasJustPressed()) {
						robot.states.gantryState = States.GantryState.RETRACTING;
					}
					break;

				case LEVEL_TWO:
					robot.lift.setHeight(10);
					if (controls.liftHeightDecreaseButton.wasJustPressed()) {
						robot.states.previousliftControlState = States.LiftControlState.LEVEL_TWO;
						robot.states.liftControlState = States.LiftControlState.LEVEL_ONE;
					}
					if (controls.liftHeightIncreaseButton.wasJustPressed()) {
						robot.states.previousliftControlState = States.LiftControlState.LEVEL_THREE;
						robot.states.liftControlState = States.LiftControlState.LEVEL_THREE;
					}

					if (controls.liftButton.wasJustPressed()) {
						robot.states.gantryState = States.GantryState.RETRACTING;
					}
					break;

				case LEVEL_THREE:
					robot.lift.setHeight(15);
					if (controls.liftHeightDecreaseButton.wasJustPressed()) {
						robot.states.previousliftControlState = States.LiftControlState.LEVEL_TWO;
						robot.states.liftControlState = States.LiftControlState.LEVEL_TWO;
					}
					if (controls.liftHeightIncreaseButton.wasJustPressed()) {
						robot.states.previousliftControlState = States.LiftControlState.CAPSTONE;
						robot.states.liftControlState = States.LiftControlState.CAPSTONE;
					}

					if (controls.liftButton.wasJustPressed()) {
						robot.states.gantryState = States.GantryState.RETRACTING;
					}
					break;

				case CAPSTONE:
					robot.lift.setHeight(13);
					if (controls.liftHeightDecreaseButton.wasJustPressed()) {
						robot.states.previousliftControlState = States.LiftControlState.LEVEL_THREE;
						robot.states.liftControlState = States.LiftControlState.LEVEL_THREE;
					}

					if (controls.liftButton.wasJustPressed()) {
						robot.states.gantryState = States.GantryState.RETRACTING;
					}
					break;

				case HOLD:
					if (controls.liftButton.wasJustPressed()) {
						robot.states.gantryState = States.GantryState.RETRACTING;
					}
					break;

				default:
					robot.states.liftControlState = States.LiftControlState.HOME;
					break;
			}

			/*Lift Control State Machine */
			switch (robot.states.liftState) {
				case IDLE:
					robot.lift.stop();

					if (gamepad2ex.getRightY() >= 0.1) {
						robot.states.liftState = States.LiftState.EXTEND;
					}
					else if (gamepad2ex.getRightY() <= -0.1) {
						robot.states.liftState = States.LiftState.RETRACT;
					}
					if (controls.liftKillButton.wasJustPressed()) {
						robot.states.liftControlState = States.LiftControlState.HOME;
						robot.states.liftState = States.LiftState.POSITION_CONTROL;
					}
					break;

				case EXTEND:
					robot.lift.setLiftPower(gamepad2ex.getRightY());

					if (gamepad2ex.getRightY() <= 0.1) {
						robot.states.liftControlState = States.LiftControlState.HOLD;
						robot.lift.setHeight(robot.lift.getHeight());
						robot.states.liftState = States.LiftState.POSITION_CONTROL;
					}
					break;

				case RETRACT:
					robot.lift.setLiftPower(gamepad2ex.getRightY());

					if (gamepad2ex.getRightY() >= -0.1) {
						robot.states.liftControlState = States.LiftControlState.HOLD;
						robot.lift.setHeight(robot.lift.getHeight());
						robot.states.liftState = States.LiftState.POSITION_CONTROL;
					}
					break;

				case POSITION_CONTROL:
					robot.lift.update();

					if (controls.liftKillButton.wasJustPressed()) {
						robot.states.liftControlState = States.LiftControlState.HOME;
						robot.states.liftState = States.LiftState.IDLE;
					}
					if (gamepad2ex.getRightY() >= 0.1) {
						robot.states.liftState = States.LiftState.EXTEND;
					}
					else if (gamepad2ex.getRightY() <= -0.1) {
						robot.states.liftState = States.LiftState.RETRACT;
					}
					break;

				default:
					robot.states.liftState = States.LiftState.IDLE;
					break;
			}

			switch (robot.states.capstoneControlState) {
				case UP:
					robot.capstone.capSetPosition(0);
					break;

				case DOWN:
					robot.capstone.capSetPosition(180);
					break;

				case POSITION_CONTROL:
					double capPosition = gamepad2.right_trigger;
					robot.capstone.capSetPosition(capPosition);
					break;

				default:
					robot.states.capstoneControlState = States.CapstoneControlState.POSITION_CONTROL;
					break;
			}

			/*Telemetry*/
			telemetry.addLine("Lift")
					.addData("Level", robot.states.previousliftControlState)
					.addData("State", robot.states.liftState);

			telemetry.addLine("Gantry")
					.addData("Position", robot.gantry.getPosition())
					.addData("State", robot.states.gantryState);

			/*telemetry.addLine("Intake")
					.addData("Red", robot.intake.freightSensor.red())
					.addData("Green", robot.intake.freightSensor.green())
					.addData("Blue", robot.intake.freightSensor.blue())
					.addData("Freight Loaded", robot.intake.isLoaded());*/

			telemetry.addLine("Position")
					.addData("x", poseEstimate.getX())
					.addData("y", poseEstimate.getY())
					.addData("heading", poseEstimate.getHeading());

			telemetry.update();

			/*Dashboard*/
			packet.put("Lift", "Telemetry");
			packet.put("Height", robot.lift.getHeight());
			packet.put("Filtered Height", robot.lift.getFilteredHeight());
			packet.put("Level", robot.states.liftControlState.name());

			packet.put("Gantry", "Telemetry");
			packet.put("Position", robot.gantry.getPosition());

			packet.put("Intake", "Telemetry");
			packet.put("Red", robot.intake.freightSensor.red());
			packet.put("Green", robot.intake.freightSensor.green());
			packet.put("Blue", robot.intake.freightSensor.blue());
			packet.put("Freight Loaded", robot.intake.isLoaded());

			packet.put("Robot Position", "Telemetry");
			packet.put("x", poseEstimate.getX());
			packet.put("y", poseEstimate.getY());
			packet.put("heading (deg)", Math.toDegrees(poseEstimate.getHeading()));

			dashboard.sendTelemetryPacket(packet);

			/*End of loop updates*/
			gamepad1ex.readButtons();
			gamepad2ex.readButtons();
			controls.readValues();
		}
	}
}
