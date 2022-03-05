package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.gantry.Gantry;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebcamBlue;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebcamRed;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebcamRedAlt;

@TeleOp(name = "Webcam Test", group = "test")
public class WebcamTest extends LinearOpMode{

	WebcamRedAlt webcam;
	Gantry gantry;

	@Override
	public void runOpMode() throws InterruptedException{
		webcam = new WebcamRedAlt(hardwareMap, telemetry);
		gantry = new Gantry(hardwareMap, telemetry, false);

		waitForStart();
		while (opModeIsActive()) {
			//webcam.getWebcamData();

			telemetry.addData("Position: ", webcam.locateDuck());
			telemetry.addData("Position: ", gantry.getPosition());
			telemetry.update();
		}
	}
}
