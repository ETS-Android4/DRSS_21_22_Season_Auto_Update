package org.firstinspires.ftc.teamcode.subsystems.spinner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;

@Config
public class Spinner {

    public DcMotorEx spinnerMotor;

    public static double spinSpeed = 0.75;

    Telemetry telemetry;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Spinner(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        spinnerMotor = map.get(DcMotorEx.class, "spinnerMotor");
        spinnerMotor.setDirection(DcMotorEx.Direction.FORWARD);
        spinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runSpinner(double speed) {
        spinnerMotor.setPower(speed);
    }

    public void stop() {
        spinnerMotor.setPower(0);
    }

}
