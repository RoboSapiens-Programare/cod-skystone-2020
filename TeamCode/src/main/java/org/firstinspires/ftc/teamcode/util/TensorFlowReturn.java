package org.firstinspires.ftc.teamcode.util;

public class TensorFlowReturn {
    public double top, left, right, bottom, approxAngle;
    public boolean hasDetectedObject;
    public String detectedObjectLabel;

    public TensorFlowReturn(double top, double left, double right, double bottom, double approxAngle, boolean detectedObject, String detectedObjectLabel) {
        this.top = top;
        this.left = left;
        this.right = right;
        this.bottom = bottom;
        this.approxAngle = approxAngle;
        this.hasDetectedObject = detectedObject;
        this.detectedObjectLabel = detectedObjectLabel;
    }
}
