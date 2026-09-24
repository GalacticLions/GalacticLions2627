package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class StraferTeleOp_Iterative extends OpMode {

    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(this);
        robot.init();
        robot.resetYaw();

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();
    }
    
    @Override
    public void loop() {
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
    }
    
    @Override
    public void stop() {
        
    }
}
