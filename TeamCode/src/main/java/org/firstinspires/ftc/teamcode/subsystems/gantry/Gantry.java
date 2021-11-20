package org.firstinspires.ftc.teamcode.subsystems.gantry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gantry{
	DcMotorEx gantryMotor;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Gantry(HardwareMap map, Telemetry telemetry){
		this.telemetry = telemetry;

		gantryMotor = map.get(DcMotorEx.class, "gantryMotor");
		gantryMotor.setDirection(DcMotorEx.Direction.FORWARD);
		gantryMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		gantryMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}
	public void setGantryPower(double power){
		gantryMotor.setPower(power);
		return;
	}
	public void setGantryVelocity(int velocity){
		gantryMotor.setVelocity(velocity);
		return;
	}
}
