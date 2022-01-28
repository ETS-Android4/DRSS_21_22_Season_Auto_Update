package org.firstinspires.ftc.teamcode.subsystems.webcam;

import java.util.Comparator;

/**
 * Created by Antoine on 1/18/2022
 */
public class BarcodeItem {
    public String objectType;
    public double horizontalPosition;

    public double getHorizontalPosition() {
        return horizontalPosition;
    }

    public void setHorizontalPosition(double leftPosition) {
        horizontalPosition = leftPosition;
    }
}
