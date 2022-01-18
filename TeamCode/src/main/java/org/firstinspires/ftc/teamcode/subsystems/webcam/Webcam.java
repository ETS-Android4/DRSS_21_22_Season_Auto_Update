package org.firstinspires.ftc.teamcode.subsystems.webcam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Antoine on 1/18/2022
 */
public class Webcam {

    private HardwareMap map;

    private static final String VUFORIA_KEY = "AUz/xEr/////AAABmZjEf3Mc1kGYkOsXVF3u1Gl8gO6qZozGZxty6mcO/xE35elxxgMBwh4/zzwC9Dh4EPKvDexbQAVpjQJzz+Cx+PMYbKiPfvJNsyHDoJkWCPC1skmjKJq/4ctLkD1zGtWPhVUsdGK9ib6ze346j5nHgoFwzoi4SAITZUfQZEj2ccyiWs3zvY2DzbL/QgXrk391epqrpmB6y96vnvCsTUYA6i1y8pg7TZmjUBNWC/3PMr0EHBAFzu+cgtMWVD2sjR9XYcyh9eCRKFNq1aZwikL2P2F4Px5eyujkCVBsnQ0N+dNBo/UCREIF2az5iJY/x+qnrr8aZ2Rj1Gri12gHuKLT7BWS73HKsC9XVURurHz9RmJs";
    public VuforiaLocalizer vuforia;

    public Webcam(HardwareMap map, Telemetry telemetry) {

        this.map = map;

        telemetry.addData("Initializing", "Vuforia");
        telemetry.update();

        initVuforia();

        telemetry.addData("Vuforia", "Initialized");
        telemetry.update();

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = map.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Start streaming to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);
    }

}
