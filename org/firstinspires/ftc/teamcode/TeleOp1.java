package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOp1 extends LinearOpMode {
    public boolean speedUpPressed = false;
    public boolean speedDownPressed = false;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Robot9330 robot = new Robot9330(this, false);
        
        int maxSpeedMultiplier = 3;
        int speedMultiplier = (int) Math.ceil(maxSpeedMultiplier / 2.0);

        waitForStart();
        
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            //change speedMultiplier
            if(gamepad1.right_bumper && speedMultiplier < maxSpeedMultiplier && !speedUpPressed) {
                speedMultiplier++;
                speedUpPressed = true;
                
                new Thread() {
                    @Override
                    public void run() {
                        robot.pause(0.25);
                        speedUpPressed = false;
                    }
                }.start();
            }
            
            if(gamepad1.left_bumper && speedMultiplier > 1 && !speedDownPressed) {
                speedMultiplier--;
                speedDownPressed = true;
                
                new Thread() {
                    @Override
                    public void run() {
                        robot.pause(0.25);
                        speedDownPressed = false;
                    }
                }.start();
            }
            
            if(gamepad1.x) robot.imu.initialize(robot.parameters);
            
            double x = gamepad1.left_stick_x * 1.1;
            double y = -gamepad1.left_stick_y; // Reversed
            double rx = gamepad1.right_stick_x;

            robot.move(x * speedMultiplier / maxSpeedMultiplier, y * speedMultiplier / maxSpeedMultiplier, rx * speedMultiplier / maxSpeedMultiplier);
            robot.liftLift(gamepad2.right_trigger - gamepad2.left_trigger);
            
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("rx", rx);
            telemetry.addData("motorLiftLeft", robot.motorLiftLeft.getCurrentPosition());
            telemetry.addData("motorLiftRight", robot.motorLiftRight.getCurrentPosition());
            telemetry.addData("power", gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.update();
        
            if(gamepad2.right_bumper) {
                robot.closeClaw();
            }
            
            if(gamepad2.left_bumper) {
                robot.openClaw();
            }
            
            if(gamepad2.x) {
                robot.servoClaw.setPosition(robot.servoClaw.getPosition() == 0 ? 0.5 : 0);
            }
        }
    }
}
