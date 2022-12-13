package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Robot9330 {
    public BNO055IMU imu;
    public DcMotor motorDriveFrontLeft;
    public DcMotor motorDriveFrontRight;
    public DcMotor motorDriveBackLeft;
    public DcMotor motorDriveBackRight;
    public DcMotor motorLiftLeft;
    public DcMotor motorLiftRight;
    public Servo servoClaw;
    
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public boolean flip;
    
    public Robot9330(HardwareMap hardwareMap, boolean flip) {
        this.flip = flip;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        motorDriveFrontLeft = hardwareMap.get(DcMotor.class, "motorDriveFrontLeft");
        motorDriveFrontRight = hardwareMap.get(DcMotor.class, "motorDriveFrontRight");
        motorDriveBackLeft = hardwareMap.get(DcMotor.class, "motorDriveBackLeft");
        motorDriveBackRight = hardwareMap.get(DcMotor.class, "motorDriveBackRight");
        motorLiftLeft = hardwareMap.get(DcMotor.class, "motorLiftLeft");
        motorLiftRight = hardwareMap.get(DcMotor.class, "motorLiftRight");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        
        //Set up IMU
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        // Reverse any motors if necessary
        //motorDriveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Turn on brakes
        motorLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void auto() {
        int signalValue = readSignal();
        
        liftLift(1, 0.05);
        move(0, 1, 0, 0.25);
        // pause(1);
        // move(-1, 0, 0, 0.5);
        // pause(1);
        // move(0, 0, -1, 0.125);
        // pause(1);
        // liftLift(1, 1.6);
        // pause(1);
        // move(-1, 1, 0, 0.25);
        // closeClaw();
        // pause(1);
        // move(1, -1, 0, 0.25);
        // pause(1);
        // move(0, 0, 1, 0.125);
    }
    
    // Reads signal and returns what number it is showing
    public int readSignal() {
        // Because we don't have the camera set up guess for a spot and hope it's the correct one
        return 1;
    }
    
    // Moves robot
    public void move(double x, double y, double rx) {
        if(flip) {
            x = -x;
            rx = -rx;
        }
        
        // Magical math
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        
        // Move motors
        motorDriveFrontLeft.setPower(frontLeftPower);
        motorDriveFrontRight.setPower(frontRightPower);
        motorDriveBackLeft.setPower(backLeftPower);
        motorDriveBackRight.setPower(backRightPower);
    }
    
    // Moves robot for an amount of time
    public void move(double x, double y, double rx, double seconds) {
        move(x, y, rx);
        pause(seconds);
        move(0, 0, 0);
    }
    
    public void liftLift(double p) {
        motorLiftLeft.setPower(p);
        motorLiftRight.setPower(p);
    }
    
    public void liftLift(double p, double seconds) {
        liftLift(p);
        pause(seconds);
        liftLift(0);
    }
    
    public void openClaw() {
        servoClaw.setPosition(0);
    }
    
    public void closeClaw() {
        servoClaw.setPosition(0.5);
    }
    
    public void pause(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch(InterruptedException exc) {}
    }
}
