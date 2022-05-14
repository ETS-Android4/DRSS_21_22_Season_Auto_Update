package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

public class HamBot{

	public MecanumDrivetrain drive;

	public HamBot(HardwareMap map) {
		drive = new MecanumDrivetrain(map);
	}
}
