package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.robot.HamBot;

@TeleOp(name = "Ham Bot TeleOp", group = "Test")
public class HamBotTeleOp extends LinearOpMode{

	HamBot robot;

	double frontLeft = 0;
	double frontRight = 0;
	double backLeft = 0;
	double backRight = 0;

	double speedModifier = 0.5;

	@Override
	public void runOpMode() throws InterruptedException{
		robot = new HamBot(hardwareMap);

		waitForStart();

		while(!isStopRequested()) {
			float gamepad1LeftY = -gamepad1.left_stick_y;        // Sets the gamepads left sticks y position to a float so that we can easily track the stick
			float gamepad1LeftX = gamepad1.left_stick_x;       // Sets the gamepads left sticks x position to a float so that we can easily track the stick
			float gamepad1RightX = -gamepad1.right_stick_x;     // Sets the gamepads right sticks x position to a float so that we can easily track the stick

			double FrontRight = -gamepad1LeftX - gamepad1LeftY + gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
			double FrontLeft = -gamepad1LeftX + gamepad1LeftY + gamepad1RightX;     // Combines the inputs of the sticks to clip their output to a value between 1 and -1
			double BackRight = gamepad1LeftX - gamepad1LeftY + gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1
			double BackLeft = gamepad1LeftX + gamepad1LeftY + gamepad1RightX;      // Combines the inputs of the sticks to clip their output to a value between 1 and -1

			//speed Controls
			if (gamepad1.left_trigger > .3){   // Do the following while the left trigger is being held down
				speedModifier = .25;                    // Sets the speed to quarter speed
			}

			if (gamepad1.right_trigger < .3 && gamepad1.left_trigger < .3){   // Do the following  while the right trigger is held down and the left trigger is not
				speedModifier = 0.5;                     // Sets the speed to half speed
			}

			if (gamepad1.right_trigger > .3){    // Do the following while the left trigger is not being held down
				speedModifier = 1;
			}

			frontLeft = Range.clip(Math.pow(FrontRight, 3), -speedModifier, speedModifier);    // Slows down the motor and sets its max/min speed to the double "speed"
			frontRight = Range.clip(Math.pow(FrontLeft, 3), -speedModifier, speedModifier);      // Slows down the motor and sets its max/min speed to the double "speed"
			backLeft = Range.clip(Math.pow(BackRight, 3), -speedModifier, speedModifier);      // Slows down the motor and sets its max/min speed to the double "speed"
			backRight = Range.clip(Math.pow(BackLeft, 3), -speedModifier, speedModifier);        // Slows down the motor and sets its max/min speed to the double "speed"

			robot.drive.setPowers(frontLeft, frontRight, backLeft, backRight);
		}

		robot.drive.stop();
	}
}
