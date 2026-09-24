package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.Constants.CannonConstants;

import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;


@Disabled
@TeleOp(name="Beta TeleOP (PID)", group="TeleOP")
public class Beta_TeleOP_PID extends OpMode {
    Limelight3A limelight;
    DcMotor frontLeft, frontRight, backLeft, backRight, leftSlide, rightSlide;
    DcMotorEx topLauncher, bottomLauncher;
    VisualServoing visualServoing;
    CRServo backLeftServo, backRightServo;
    DcMotor frontIntake;
    IMU imu;
    Double intakePower;

    Double LAUNCH_VELOCITY;
    Double kP;
    Double kI;
    Double kD;
    Double kF;


    @Override
    public void init() {
        //Constants Init
        //Cannon Constants
        intakePower = CannonConstants.IntakePower;
        kP = CannonConstants.kP;
        kI = CannonConstants.kI;
        kD = CannonConstants.kD;
        kF = CannonConstants.kF;

        // Set target velocities in Ticks Per Second. These are example values and will need tuning.
        LAUNCH_VELOCITY = CannonConstants.FRONT_LAUNCH_VELOCITY;

        //Limelight Init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(90);
        limelight.start();

        //Motors Init
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        topLauncher = hardwareMap.get(DcMotorEx.class, "TLaunch");
        bottomLauncher = hardwareMap.get(DcMotorEx.class, "BLaunch");
        frontIntake = hardwareMap.get(DcMotor.class, "Intake");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        topLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to use encoders for velocity control. This is a crucial step.
        topLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Improved PIDF coefficients for more stable velocity control
        // P: Proportional gain - increased for faster response
        // I: Integral gain - added to eliminate steady-state error
        // D: Derivative gain - added to reduce overshoot and oscillation
        // F: Feed-forward gain - tuned for velocity control
        // Tune these values based on your motor's behavior:
        // - If oscillating: decrease P, increase D
        // - If slow to reach target: increase P, increase F
        // - If steady-state error: increase I (start small, like 0.1-0.5)


        topLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        bottomLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        // Set zero power behavior to FLOAT for launchers (reduces resistance)
        topLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        backLeftServo = hardwareMap.get(CRServo.class, "LServo");
        backRightServo = hardwareMap.get(CRServo.class, "RServo");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.FORWARD);


        //IMU Init for Field-Centric
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        //Visual Servoing Init
        visualServoing = new VisualServoing(limelight, frontLeft, frontRight, backRight, backLeft, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        kP = CannonConstants.kP;
        kI = CannonConstants.kI;
        kD = CannonConstants.kD;


        //VisualServoing: gamepad1.a
        //Check if VisualServoing
        boolean isVisualServoing = gamepad1.a;
        if (isVisualServoing) {
            visualServoing.visualServo();
        }


        //Cannon Control - Now using setVelocity for closed-loop control
        if (gamepad2.left_stick_y > 0.3) {
            topLauncher.setVelocity(-LAUNCH_VELOCITY);
            bottomLauncher.setVelocity(LAUNCH_VELOCITY);
        } else if (gamepad2.left_stick_y < -0.3) {
            topLauncher.setVelocity(LAUNCH_VELOCITY);
            bottomLauncher.setVelocity(-LAUNCH_VELOCITY);
        } else {
            topLauncher.setVelocity(0);
            bottomLauncher.setVelocity(0);
        }



        if (gamepad2.right_stick_y > 0.3) {
            backLeftServo.setPower(-intakePower);
            backRightServo.setPower(-intakePower);
            frontIntake.setPower(-intakePower);
        } else if (gamepad2.right_stick_y < -0.3) {
            backLeftServo.setPower(intakePower);
            backRightServo.setPower(intakePower);
            frontIntake.setPower(.7);
        }else {
            backLeftServo.setPower(0);
            backRightServo.setPower(0);
            frontIntake.setPower(0);
        }

        if (gamepad2.dpad_down){
            backLeftServo.setPower(intakePower-0.3);
            backRightServo.setPower(intakePower-0.3);

        }else if (gamepad2.dpad_up){
            backLeftServo.setPower(-intakePower+0.3);
            backRightServo.setPower(-intakePower+0.3);
        }



        //===========================================

        // Field-Centric Drive Code (skip if Visual Servoing is active)
        if (!isVisualServoing) {
            double y = -gamepad1.left_stick_y; // Forward/Backward (reversed)
            double x = gamepad1.left_stick_x * 1.1; // Strafe Left/Right (counteract imperfect strafing)
            double rx = gamepad1.right_stick_x; // Rotation


            // Reset IMU yaw with guide button
            if (gamepad1.guide) {
                imu.resetYaw();
            }

            // Get robot heading from IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate movement direction based on robot's heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            if (!gamepad1.left_bumper) {
                // Set motor powers (adjust multiplier as needed)
                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);
            } else {
                frontLeft.setPower(frontLeftPower * 0.5);
                backLeft.setPower(backLeftPower * 0.5);
                frontRight.setPower(frontRightPower * 0.5);
                backRight.setPower(backRightPower * 0.5);
            }
        }

    }
    @Override
    public void stop(){
        limelight.stop();
    }
}

