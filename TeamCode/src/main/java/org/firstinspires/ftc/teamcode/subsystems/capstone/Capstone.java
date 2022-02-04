package org.firstinspires.ftc.teamcode.subsystems.capstone;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Antoine on 2/3/2022
 */
public class Capstone {

    public Servo capServo;

    int DEFAULT_MIN_ANGLE = 0;
    int DEFAULT_MAX_ANGLE = 180;

    Telemetry telemetry;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Capstone(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        capServo = map.get(Servo.class, "capServo");

        capSetPosition(0);
    }

    public void capSetPosition(double position) {
        capServo.setPosition(position);
    }
}
