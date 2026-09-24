package org.firstinspires.ftc.team5898.LimelightUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team5898.Constants.LimelightConstants;

import java.util.List;

@Configurable
public class VisualServoing {
    // Horizontal (tx) alignment - turning left/right
    public static final double Kp_HEADING = 0.02; // Proportional gain for horizontal alignment
    public static final double MIN_COMMAND_HEADING = 0.10;
    public static final double HEADING_THRESHOLD = 0.05; // Horizontal offset threshold (degrees)
    public static final double MAX_TURN_POWER = 0.5;

    // Vertical (ty) alignment - moving forward/backward
    public static final double Kp_DISTANCE = 0.03; // Proportional gain for vertical alignment
    public static final double MIN_COMMAND_DISTANCE = 0.12;
    public static final double DISTANCE_THRESHOLD = 1.0; // Vertical offset threshold (degrees)
    public static final double MAX_DRIVE_POWER = 0.5;

    // Target values
    public static double TARGET_TY = 0.0; // Target ty value (adjust based on desired distance)
    public static double TARGET_TX = 0.0; // Target tx value (default 0.0 = center)
    public static int TagID = -1;
    public static double targetAreaThreshold = LimelightConstants.targetAreaThreshold;
    private final Limelight3A limelight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final Telemetry telemetry;
    private static double BLUE_TX = LimelightConstants.BLUE_ALLIANCE_TX;
    private static double RED_TX = LimelightConstants.RED_ALLIANCE_TX;

    // Constructor

    public VisualServoing(Limelight3A limelight, DcMotor frontLeft, DcMotor frontRight, DcMotor backRight,
                          DcMotor backLeft, Telemetry telemetry) {
        this.limelight = limelight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;
    }

    /**
     * Returns true when the robot is aligned to the target (both tx and ty errors within thresholds).
     * Returns false if there is no valid target or alignment is not yet complete.
     */
    public boolean isAligned() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) return false;

        boolean targetFound = false;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int id = fr.getFiducialId();
            if (id == 20 || id == 24) { targetFound = true; break; }
        }
        if (!targetFound) return false;

        double tx = result.getTx();
        double ty = result.getTy();
        double ta = result.getTa();

        double effectiveTargetTX;
        if (ta <= targetAreaThreshold) {
            if (TagID == 20) effectiveTargetTX = BLUE_TX;
            else if (TagID == 24) effectiveTargetTX = RED_TX;
            else effectiveTargetTX = TARGET_TX;
        } else {
            effectiveTargetTX = 0.0;
        }

        double txError = tx - effectiveTargetTX;
        double tyError = ty - TARGET_TY;

        return Math.abs(txError) <= HEADING_THRESHOLD && Math.abs(tyError) <= DISTANCE_THRESHOLD;
    }

    public void visualServo() {
        targetAreaThreshold = LimelightConstants.targetAreaThreshold;
        BLUE_TX = LimelightConstants.BLUE_ALLIANCE_TX;
        RED_TX = LimelightConstants.RED_ALLIANCE_TX;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            telemetry.addData("No valid result", "");
            telemetry.update();
            return;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            telemetry.addData("No fiducials found", "");
            telemetry.update();
            return;
        }

        boolean targetFound = false;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int fiducialId = fr.getFiducialId();
            if (fiducialId == 20 || fiducialId == 24) {
                targetFound = true;
                TagID = fiducialId;
                break;
            }
        }

        if (!targetFound) {
            TagID = -1;
        }

        if (targetFound) {
            double tx = result.getTx(); // Horizontal offset from crosshair
            double ty = result.getTy(); // Vertical offset from crosshair
            double ta = result.getTa(); // Target Area to determine distance

            // --- TX Determination by using distance ---
            if (ta <= targetAreaThreshold) {
                if (TagID == 20) TARGET_TX = BLUE_TX;
                else if (TagID == 24) TARGET_TX = RED_TX;
            } else {
                TARGET_TX = 0.0;
            }

            // --- TX (Heading) Control ---
            // Calculate error relative to TARGET_TX
            double txError = tx - TARGET_TX;
            double steeringAdjust = 0.0;

            if (Math.abs(txError) > HEADING_THRESHOLD) {
                // Proportional control based on error
                steeringAdjust = Kp_HEADING * txError;

                // Add minimum command to overcome friction
                if (txError > 0) {
                    steeringAdjust += MIN_COMMAND_HEADING;
                } else {
                    steeringAdjust -= MIN_COMMAND_HEADING;
                }
                steeringAdjust = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, steeringAdjust));
            }

            // --- TY (Distance) Control ---
            double tyError = ty - TARGET_TY;
            double driveAdjust = 0.0;

            // DISABLED by default: Set to true if you want distance control
            boolean enableDistanceControl = false;

            if (enableDistanceControl && Math.abs(tyError) > DISTANCE_THRESHOLD) {
                driveAdjust = Kp_DISTANCE * tyError;

                if (tyError > 0) {
                    driveAdjust += MIN_COMMAND_DISTANCE;
                } else {
                    driveAdjust -= MIN_COMMAND_DISTANCE;
                }
                driveAdjust = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, driveAdjust));
            }

            // Combine turning and driving
            double frontLeftPower = driveAdjust + steeringAdjust;
            double backLeftPower = driveAdjust + steeringAdjust;
            double frontRightPower = driveAdjust - steeringAdjust;
            double backRightPower = driveAdjust - steeringAdjust;

            // Normalize powers if they exceed 1.0
            double maxPower = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                backLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Apply motor powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);
            telemetry.addData("txError", txError);
            telemetry.addData("steeringAdjust", steeringAdjust);
            telemetry.addData("driveAdjust", driveAdjust);
            telemetry.addData("Heading Aligned", Math.abs(txError) <= HEADING_THRESHOLD);
            telemetry.addData("Distance Aligned", Math.abs(tyError) <= DISTANCE_THRESHOLD);
            telemetry.update();
        } else {
            // Stop all motors when no valid target
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            telemetry.addData("No valid result", "");
            telemetry.update();
        }
    }
}