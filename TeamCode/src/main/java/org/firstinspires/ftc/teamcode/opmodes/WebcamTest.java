package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;

@TeleOp(name = "Webcam Test", group = "test")
public class WebcamTest extends LinearOpMode{

	Webcam webcam;

	@Override
	public void runOpMode() throws InterruptedException{
		webcam = new Webcam(hardwareMap, telemetry);

		waitForStart();
		while (opModeIsActive()) {
			//webcam.getWebcamData();

			telemetry.addData("Position: ", webcam.locateDuck());
			telemetry.update();
		}
	}
}
