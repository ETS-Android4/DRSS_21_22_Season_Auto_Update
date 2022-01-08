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
	public TriggerReader outtakeTrigger;

	public ButtonReader gantryForwardButton;
	public ButtonReader gantryReverseButton;

	public ButtonReader pusherExtendButton;
	public ButtonReader pusherRetractButton;

	public ButtonReader liftButton;

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

		outtakeTrigger = new TriggerReader(
				gamepad2ex, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

		/*Gantry Controls*/
		gantryForwardButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.DPAD_UP
		);

		gantryReverseButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.DPAD_DOWN
		);

		/*Pusher Controls*/
		pusherExtendButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.A
		);
		pusherRetractButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.B
		);

		/*Lift Controls*/
		liftButton = new ButtonReader(
				gamepad2ex, GamepadKeys.Button.START
		);

	}

	public void readValues() {
		speedTrigger.readValue();
		halfSpeedButton.readValue();
		fullSpeedButton.readValue();

		driveFlipButton.readValue();

		intakeTrigger.readValue();
		outtakeTrigger.readValue();

		gantryForwardButton.readValue();
		gantryReverseButton.readValue();

		pusherExtendButton.readValue();
		pusherRetractButton.readValue();

		liftButton.readValue();
	}
}
