package org.firstinspires.ftc.team5898;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.team5898.Constants.CannonConstants;
import org.firstinspires.ftc.team5898.LimelightUtils.VisualServoing;

@Autonomous(name = "Beta Auto", group = "Beta", preselectTeleOp = "Beta TeleOP (PID)")
public class Beta_Auto_Limelight extends LinearOpMode {
    // variable declaration & setup
    DcMotor frontLeft, frontRight, backLeft, backRight,leftSlide,rightSlide, frontIntake;
    DcMotorEx topLauncher, bottomLauncher;


    // motor counts per rotation (ticks/pulses per rotation)
    // check motor specs from manufacturer
    // 537.7 is for GoBilda 312 RPM Yellow Jacket motor
    double cpr = 537.7;

    // adjust gearRatio if you have geared up or down your motors
    double gearRatio = 1;

    // wheel diameter in inches
    // 3.779 is for the GoBilda mecanum wheels
    double diameter = 3.779;

    // counts per inch: cpr * gear ratio / (pi * diameter (in inches))
    double cpi = (cpr * gearRatio) / (Math.PI * diameter);

    // use calibrate auto to check this number before proceeding
    double bias = 1.0; // adjust based on calibration opMode

    double strafeBias = 0.9;// change to adjust only strafing movement
    //
    double conversion = cpi * bias;
    final double LaunchVelocity = CannonConstants.FRONT_LAUNCH_VELOCITY;
    double intakePower = CannonConstants.IntakePower;
    IMU imu;
    Limelight3A limelight;
    String side_switch;
    CRServo backLeftServo, backRightServo;

    double LAUNCH_VELOCITY;

    VisualServoing visualServoing;

    Double P = CannonConstants.kP;
    Double I = CannonConstants.kI;
    Double D = CannonConstants.kD;
    Double F = CannonConstants.kF;

    @Override
    public void runOpMode() {
        //Constants Init
        //Cannon Constants
        intakePower = CannonConstants.IntakePower;

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
//        leftSlide = hardwareMap.get(DcMotor.class, "LS");
//        rightSlide = hardwareMap.get(DcMotor.class, "RS");
        topLauncher = hardwareMap.get(DcMotorEx.class, "TLaunch");
        bottomLauncher = hardwareMap.get(DcMotorEx.class, "BLaunch");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        topLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to use encoders for velocity control. This is a crucial step.
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

        //TODO: Tune PID for new Robot
        topLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));
        bottomLauncher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));

        // Set zero power behavior to FLOAT for launchers (reduces resistance)
        topLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntake = hardwareMap.get(DcMotor.class, "Intake");
        backLeftServo = hardwareMap.get(CRServo.class, "LServo");
        backRightServo = hardwareMap.get(CRServo.class, "RServo");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.FORWARD);

        visualServoing = new VisualServoing(limelight, frontLeft, frontRight, backRight, backLeft, telemetry);

        // wait for Start to be pressed
        //TODO: for new robot
        while(!isStarted()){
            telemetry.addLine("Press A for blue, press B for red (Gamepad 1)");
            telemetry.update();
            if(side_switch==null) {
                if (gamepad1.a) {
                    side_switch = "blue";
                    telemetry.addLine("You picked blue");
                    telemetry.update();
                } else if (gamepad1.b) {
                    side_switch = "red";
                    telemetry.addLine("You picked red");
                    telemetry.update();
                }
            }
            else{
                telemetry.addData("Alliance selected:",side_switch);
                telemetry.update();
                break;
            }
        }
        waitForStart();
//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        initGyro();
        backAndStartLauncher(33,.6);
        sleep(500);
        quickAlign(2);
        setServoPower(1);
        sleep(5000);
        topLauncher.setPower(0);
        bottomLauncher.setPower(0);
        setServoPower(0);
        sleep(500);

        if("red".equals(side_switch)) {
            turnRight(-40, .5);
            sleep(500);
            strafeRight(25, 0.5);
            setServoPower(1);
            forward(37,.3);
            sleep(100);
            setServoPower(0);
            back(35,.5);
            strafeLeft(24.5, 0.5);
            turnLeft(-40,.5);
            sleep(500);
            setServoPower(-.1);
            sleep(50);
            topLauncher.setVelocity(-LAUNCH_VELOCITY);
            bottomLauncher.setPower(-LAUNCH_VELOCITY);
            forward(5,.5);
            quickAlign(1);
            setServoPower(1);
            sleep(5000);
            strafeLeft(15, 1);

        }else if("blue".equals(side_switch)){
            turnLeft(-40, .5);
            sleep(500);
            strafeLeft(24.5, 0.5);
            setServoPower(1);
            forward(40, .5);
            sleep(100);
            setServoPower(0);
            back(35,.5);
            strafeRight(24.5, 0.5);
            turnRight(-40,.5);
            sleep(500);
            setServoPower(-.1);
            sleep(50);
            topLauncher.setVelocity(-LAUNCH_VELOCITY);
            bottomLauncher.setPower(-LAUNCH_VELOCITY);
            forward(5,.5);
            quickAlign(1);
            setServoPower(1);
            sleep(5000);
            strafeRight(15, 1);
        }
        sleep(1000);
        stop();

    }
    public void setServoPower(double power){
        frontIntake.setPower(power);
        backLeftServo.setPower(power);
        backRightServo.setPower(power);
    }

    /**
     * Use visual servoing to align with AprilTag
     * @param timeoutSeconds Maximum time to spend aligning
     */
    public void alignWithAprilTag(double timeoutSeconds) {
        double startTime = getRuntime();

        telemetry.addLine("Starting AprilTag alignment...");
        telemetry.update();

        while (opModeIsActive() && (getRuntime() - startTime) < timeoutSeconds) {
            // Use the visual servoing to align
            visualServoing.visualServo();

            // Check if we're aligned (you can adjust these thresholds)
            if (isAlignedWithTarget()) {
                telemetry.addLine("Target aligned!");
                telemetry.update();
                break;
            }

            sleep(50); // Small delay to prevent overwhelming the system
        }

        // Stop all motors after alignment
        stopDriveMotors();

        if ((getRuntime() - startTime) >= timeoutSeconds) {
            telemetry.addLine("Alignment timeout reached");
        } else {
            telemetry.addLine("Alignment complete");
        }
        telemetry.update();
    }

    /**
     * Check if the robot is aligned with the target
     * @return true if aligned within acceptable thresholds
     */
    public boolean isAlignedWithTarget() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Horizontal offset
            double ty = result.getTy(); // Vertical offset

            // Use the same thresholds as in VisualServoing class
            boolean horizontallyAligned = Math.abs(tx) <= VisualServoing.HEADING_THRESHOLD;
            boolean verticallyAligned = Math.abs(ty - VisualServoing.TARGET_TY) <= VisualServoing.DISTANCE_THRESHOLD;

            telemetry.addData("tx (horizontal)", tx);
            telemetry.addData("ty (vertical)", ty);
            telemetry.addData("Horizontally Aligned", horizontallyAligned);
            telemetry.addData("Vertically Aligned", verticallyAligned);

            // For this auto, we primarily care about horizontal alignment
            return horizontallyAligned;
        }
        return false;
    }

    /**
     * Stop all drive motors
     */
    public void stopDriveMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void backAndStartLauncher(double inches, double speed) {
        topLauncher.setVelocity(-LaunchVelocity);
        bottomLauncher.setPower(-LaunchVelocity);

        back(inches, speed);
    }



    /**
     * Simple visual servoing alignment that can be called anywhere in autonomous
     * This is a simpler version that just aligns horizontally
     * @param maxTime Maximum time in seconds to spend aligning
     */
    public void quickAlign(double maxTime) {
        alignWithAprilTag(maxTime);
    }


    /**
     * Use to make the robot go forward a number of inches
     *
     * @param inches distance to travel in inches
     * @param speed  has a range of [0,1]
     */
    public void forward(double inches, double speed) {
        moveToPosition(inches, speed);
    }

    /**
     * Use to make the robot go backward a number of inches
     *
     * @param inches distance to travel in inches
     * @param speed  has a range of [0,1]
     */
    public void back(double inches, double speed) {
        moveToPosition(-inches, speed);
    }

    /**
     * Rotate the robot left
     *
     * @param degrees the amount of degrees to rotate
     * @param speed   has a range of [0,1]
     */
    public void turnLeft(double degrees, double speed) {
        turnWithGyro(degrees, -speed);
    }

    /**
     * Rotate the robot right
     *
     * @param degrees the amount of degrees to rotate
     * @param speed   has a range of [0,1]
     */
    public void turnRight(double degrees, double speed) {
        turnWithGyro(degrees, speed);
    }

    /**
     * Strafe left
     *
     * @param inches the distance in inches to strafe
     * @param speed  has a range of [0,1]
     */
    public void strafeLeft(double inches, double speed) {
        strafeToPosition(-inches, speed);
    }

    /**
     * Strafe right
     *
     * @param inches the distance in inches to strafe
     * @param speed  has a range of [0,1]
     */
    public void strafeRight(double inches, double speed) {
        strafeToPosition(inches, speed);
    }

    /*
     * This function's purpose is simply to drive forward or backward.
     * To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * conversion));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Busy...", "");
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /*
     * This function uses the Hub IMU Integrated Gyro to turn a precise number of
     * degrees (+/- 5).
     * Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection) {
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Initialize

        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // make this negative?
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        double first;
        double second;

        // turning right
        if (speedDirection > 0) {
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
            } else {
                first = devertify(yaw);
            }
            second = degrees + devertify(yaw);
        }

        // turning left
        else {
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
            } else {
                first = devertify(yaw);
            }
            second = devertify(-degrees + devertify(yaw));
        }

        // Go to position
        double firsta = convertify(first - 5);
        double firstb = convertify(first + 5);
        turnWithEncoder(speedDirection);

        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {// within range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        } else {
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {// within range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }

        double seconda = convertify(second - 5);// 175
        double secondb = convertify(second + 5);// -175
        turnWithEncoder(speedDirection / 3);

        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {// within range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {// within
                // range?
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw = robotOrientation.getYaw(AngleUnit.DEGREES); // make this negative?
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
     * This function uses the encoders to strafe left or right.
     * Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed) {
        int move = (int) (Math.round(inches * cpi * strafeBias));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Working...", " ");
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /*
     * These functions are used in the turnWithGyro function to ensure inputs
     * are interpreted properly.
     */
    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 360) {
            degrees = degrees - 360;
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 179) {
            degrees = -(360 - degrees);
        }
        return degrees;
    }

    /*
     * This function is called at the beginning of the program to activate
     * the IMU Integrated Gyro.
     */
    public void initGyro() {
        // Check the orientation of the Rev Hub
        // more info on ftc-docs.firstinspires.org
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    /*
     * This function is used in the turnWithGyro function to set the
     * encoder mode and turn.
     */
    public void turnWithEncoder(double input) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontLeft.setPower(input);
        backLeft.setPower(input);
        frontRight.setPower(-input);
        backRight.setPower(-input);
    }

}
