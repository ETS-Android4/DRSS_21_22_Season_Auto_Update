package org.firstinspires.ftc.teamcode.subsystems.spinner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spinner {

    public DcMotorEx spinnerMotor;

    Telemetry telemetry;
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Spinner(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        spinnerMotor = map.get(DcMotorEx.class, "spinnerMotor");
        spinnerMotor.setDirection(DcMotorEx.Direction.FORWARD);
        spinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Spinner", "Initialized");
        telemetry.update();

        packet.put("Spinner", "Initialized");
        dashboard.sendTelemetryPacket(packet);
    }

    public void runSpinner(double speed) {
        spinnerMotor.setPower(speed);
    }

    public void stop() {
        spinnerMotor.setPower(0);
    }

}
