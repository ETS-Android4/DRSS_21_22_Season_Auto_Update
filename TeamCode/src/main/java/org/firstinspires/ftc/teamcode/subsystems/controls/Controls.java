package org.firstinspires.ftc.teamcode.subsystems.controls;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Controls{

	public TriggerReader speedTrigger;
	public ButtonReader halfSpeedButton;
	public ButtonReader fullSpeedButton;

	public ButtonReader driveFlipButton;

	public TriggerReader intakeTrigger;
	public ButtonReader outtakeButton;

	public ButtonReader gantryForwardButton;
	public ButtonReader gantryReverseButton;

	public ButtonReader pusherButton;

	public ButtonReader liftKillButton;
	public ButtonReader liftButton;
	public ButtonReader liftHeightIncreaseButton;
	public ButtonReader liftHeightDecreaseButton;

	public TriggerReader capstoneTrigger;

	public Controls(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {

		/*Speed Controls*/
		speedTrigger = new TriggerReader(
				gamepad1ex, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

		halfSpeedButton = new ButtonReader(
				gamepad1ex, GamepadKeys.Button.X
		);

		fullSpeedButton = new ButtonReader(
				gamepad1ex, GamepadKeys.Button.Y
		);

		/*Drivetrain Direction Controls*/
		driveFlipButton = new ButtonReader(
				gamepad1ex, GamepadKeys.Button.START
		);

		/*Intake Controls*/
		intakeTrigger = new TriggerReader(
				gamepad2ex, GamepadKeys.Trigger.LEFT_TRIGGER
		);

		outtakeButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.LEFT_BUMPER
		);

		/*Pusher Controls*/
		pusherButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.A
		);

		/*Lift Controls*/
		liftKillButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.START
		);
		liftButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.B
		);
		liftHeightIncreaseButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.DPAD_UP
		);
		liftHeightDecreaseButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.DPAD_DOWN
		);

		/*Capstone Controls*/
		capstoneTrigger = new TriggerReader(
				gamepad2ex, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

	}

	public void readValues() {
		speedTrigger.readValue();
		halfSpeedButton.readValue();
		fullSpeedButton.readValue();

		driveFlipButton.readValue();

		intakeTrigger.readValue();
		outtakeButton.readValue();

		pusherButton.readValue();

		liftKillButton.readValue();
		liftButton.readValue();
		liftHeightIncreaseButton.readValue();
		liftHeightDecreaseButton.readValue();

		capstoneTrigger.readValue();
	}
}
