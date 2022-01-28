package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;

public class WebcamTest extends LinearOpMode{

	Webcam webcam;

	@Override
	public void runOpMode() throws InterruptedException{
		webcam = new Webcam(hardwareMap, telemetry);

		waitForStart();
		while (opModeIsActive()) {
			webcam.getWebcamData();
			webcam.updateTFODetections();

			for (String label : webcam.barcodeItemLabels) {
				telemetry.addData("Item:", label);
			}
			telemetry.addData("Position: ", webcam.getDuckPosition());
			telemetry.update();
		}
	}
}
