package org.firstinspires.ftc.teamcode.subsystems.capstone;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Antoine on 2/3/2022
 */
public class Capstone {

    ServoEx capServo;

    int DEFAULT_MIN_ANGLE = 0;
    int DEFAULT_MAX_ANGLE = 180;

    Telemetry telemetry;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Capstone(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        capServo = new SimpleServo(map, "capServo",
                DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE,
                AngleUnit.DEGREES);

        capSetPosition(0);
    }

    public void capSetPosition(float angle) {
        capServo.turnToAngle(angle);
    }

    public void capSetPosition(double position) {
        capServo.setPosition(position);
    }

    public void capMove(float angle) {
        capServo.rotateByAngle(angle);
    }

    public void capMove(double distance) {
        capServo.rotateBy(distance);
    }
}
