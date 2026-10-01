/**
 * @Author: Eli Xiao
 */
package org.firstinspires.ftc.team5898.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drivetrain and IMU. Create one in an OpMode's init and use it to drive.
 */
public class Robot {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;


    /**
     * Gets the drive motors and IMU from the hardware map and sets them up.
     *
     * @param hardwareMap the OpMode's hardware map
     */
    public Robot(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get(Constants.Hardware.MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.dcMotor.get(Constants.Hardware.MOTOR_FRONT_RIGHT);
        backLeft = hardwareMap.dcMotor.get(Constants.Hardware.MOTOR_BACK_LEFT);
        backRight = hardwareMap.dcMotor.get(Constants.Hardware.MOTOR_BACK_RIGHT);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: Hub orientation must match how the hub is mounted, or field-centric drive is off
        imu = hardwareMap.get(IMU.class, Constants.Hardware.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    }

    /**
     * Drives the robot relative to itself. All inputs are clipped together so no motor goes over full power.
     *
     * @param forward power to drive forward, from -1 to 1
     * @param strafe power to strafe right, from -1 to 1
     * @param turn power to turn clockwise, from -1 to 1
     */
    public void drive(double forward, double strafe, double turn) {
        double denominator = Math.max(1.0,
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        frontRight.setPower((forward - strafe - turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);
    }


    /**
     * Drives the robot relative to the field, using the IMU heading.
     *
     * @param forward power to drive away from the driver, from -1 to 1
     * @param strafe power to strafe right, from -1 to 1
     * @param turn power to turn clockwise, from -1 to 1
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeadingRad();
        double rotatedForward = forward * Math.cos(heading) + strafe * Math.sin(heading);
        double rotatedStrafe = strafe * Math.cos(heading) - forward * Math.sin(heading);
        drive(rotatedForward, rotatedStrafe, turn);
    }

    /**
     * Reads the robot's heading from the IMU.
     *
     * @return heading in radians
     */
    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Sets the current facing as heading zero.
     */
    public void resetHeading() {
        imu.resetYaw();
    }
}
