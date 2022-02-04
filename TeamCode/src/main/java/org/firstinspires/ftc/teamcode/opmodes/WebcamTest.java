package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.gantry.Gantry;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;

@TeleOp(name = "Webcam Test", group = "test")
public class WebcamTest extends LinearOpMode{

	Webcam webcam;
	Gantry gantry;

	@Override
	public void runOpMode() throws InterruptedException{
		webcam = new Webcam(hardwareMap, telemetry);
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
