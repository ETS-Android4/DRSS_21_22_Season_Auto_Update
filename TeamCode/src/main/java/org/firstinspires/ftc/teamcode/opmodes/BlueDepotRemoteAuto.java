package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;

/**
 * Created by Antoine on 2/3/2022
 */

@Autonomous(name = "Blue Depot Remote", group = "Autonomous")
public class BlueDepotRemoteAuto extends LinearOpMode {

    CompRobot robot;
    Webcam webcam;

    int duckPosition = 1;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new CompRobot(hardwareMap, telemetry, true);
        webcam = new Webcam(hardwareMap, telemetry);

        /*Pre-Start/Post-Init Loop*/
        while (!opModeIsActive()) {
            duckPosition = webcam.locateDuck();
            telemetry.addData("Position: ", duckPosition);
            packet.put("Position: ", duckPosition);

            telemetry.addData("Robot", "Initialized");
            packet.put("Robot", "Initialized");

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }

        while (opModeIsActive()) {

        }

    }
}
