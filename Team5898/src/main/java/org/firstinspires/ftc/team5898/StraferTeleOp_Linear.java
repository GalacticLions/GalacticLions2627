package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * <h1>Srafer Teleoperation Mode</h1>
 *
 * <p>
 * </p>
 *
 * <h2>Key Capabilities</h2>
 * <ul>
 *     <li></li>
 * </ul>
 */
@TeleOp(name="StraferTeleOp", group="TeleOp")
// @Disabled
public class StraferTeleOp_Linear extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {
        //region Hardware Setup and Initialization
        robot = new RobotHardware(this);
        robot.init();
        robot.resetYaw();

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses START)
        waitForStart();

        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region Primary Driver Controls (gamepad1)
            // Read joystick inputs and apply deadzone to prevent drift from minor stick movement
            double y  = robot.applyJoystickDeadzone(-gamepad1.left_stick_y); // Forward/backward
            double x  = robot.applyJoystickDeadzone(gamepad1.left_stick_x);  // Strafe left/right
            double rx = robot.applyJoystickDeadzone(gamepad1.right_stick_x); // Rotation

            // Drive the robot using field-centric control
            robot.driveFieldCentric(x, y, rx);

            // Reset the IMU heading to zero manually by pressing the 'guide' button
            // Useful for correcting drift or re-aligning the robot to the field orientation
            if (gamepad1.guide) {
                robot.resetYaw();
            }
            //endregion

            //region Secondary Driver Controls (gamepad2)
            //endregion
        }
    }
}
