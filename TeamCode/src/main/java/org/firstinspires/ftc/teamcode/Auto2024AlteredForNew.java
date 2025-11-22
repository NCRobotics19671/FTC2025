//   ******Front******
//   * 0           1 *
//   *               *
//   *               *
//   *               *
//   * 2           3 *
//   ******Back*******

// Motor 0 --- Pin 0 --- motorFrontLeft
// Motor 1 --- Pin 1 --- motorFrontRight
// Motor 2 --- Pin 2 --- motorBackLeft
// Motor 3 --- Pin 3 --- motorBackRight

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous

public class Auto2024AlteredForNew extends OpMode {
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorBackRight = null;
    private DcMotor Arm = null;

    private DcMotor wheel = null;
    private DcMotor Alien = null;

    private Servo Claw = null;
    DistanceSensor distance;
    double kP = 0.5; // to be tuned
    double kI = 0.15;  // to be tuned]
    double kD = 0.00;  // to be tuned

    // PID variables
    double integral = 0;
    double previousError = 0;
    double deriv = 1;

    double dist = 0;

    double dist1 = 0;

    double dist2 = 0;

    double dist3 = 0;

    double distl = 0;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation;
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;


    double powerCoef = 0.75;

    boolean toggleA = false;

    double circumference = Math.PI * 2.95276;

    double MOTOR_TICK_COUNTS = 336;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 3; // Up
        usbFacingDirectionPosition = 0; // Forward

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;


        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        Arm = hardwareMap.dcMotor.get("Arm");
        Alien = hardwareMap.dcMotor.get("Alien");
        Claw = hardwareMap.get(Servo.class, "Claw");
        wheel = hardwareMap.dcMotor.get("wheel");
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ///motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        driveYdir(-15,0.5);
        Alien.setPower(-0.75);
        double t = getRuntime() + 1;
        while(getRuntime() < t){}
        Arm.setPower(-1);
        wheel.setPower(1);
        t = getRuntime() + 3;
        while(getRuntime() < t){}
        Alien.setPower(0);
        Arm.setPower(0);
        wheel.setPower(0);
        driveXdir(-10,0.8);



        //drop off sample






    }

    @Override
    public void loop() {

        distl = distance.getDistance(DistanceUnit.INCH);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Yaw (Z)", "%.2f Rad. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Alien", Alien.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
        telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
        telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
        telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }


    @Override
    public void stop() {
    }

    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }

    public void turnToAngle(double targetAngle) {
        double turnco = 0;
        targetAngle = targetAngle * 0.85714285714;
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double botheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (botheading > targetAngle) {
            turnco = -1;
        }
        else if (botheading < targetAngle){turnco = 1;}

        motorBackLeft.setPower(turnco * 0.5);
        motorBackRight.setPower(-turnco * 0.5);
        motorFrontLeft.setPower(turnco * 0.5);
        motorFrontRight.setPower(-turnco * 0.5);
        if (turnco == -1){
            while (botheading > targetAngle)
            {botheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}}
        else if (turnco == 1){ while (botheading < targetAngle)
        {botheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}}
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        /*double t = getRuntime();
        double error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double derivative;
        while (Math.abs(error) > 0.5) { // 1 degree tolerance
            boolean ready = false;
            error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            integral += error * 0.01; // Assume loop time of 0.01 seconds
            derivative = (error - previousError) / 0.01;

            double power = Math.min((kP * error + kI * integral + kD * derivative),1);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorFrontRight.setPower(power);

            previousError = error;

            telemetry.addData("Error", error);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Power", power);
            telemetry.update();

            while (!ready) {
                if (getRuntime() - t >= 0.01) {
                    ready = true;
                    telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                }
            }
            // Stop the motors
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
*/

    }


    // Reverse the right side motors
    // Reverse left motors if you are using NeveRests


    public void driveYdir(double distance, double power) {
        //
        double sign = distance/Math.abs(distance);
        distance = Math.abs(distance);
        distance = distance;
        double rpm = power * 500;
        double speed = ((rpm*3*Math.PI)/60);
        double inverse = 1/speed;
        double addtime = distance * inverse;
        //double StrafeRotations = 30/circumference;
        //int StrafeDrivingTarget =  (int)(StrafeRotations*MOTOR_TICK_COUNTS);
        double startt = getRuntime();
        double t = getRuntime();
        power = sign *power;
        double velocity = -(rpm * (1/60) * 336);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        motorFrontLeft.setVelocity(-velocity);
        motorBackLeft.setVelocity(-velocity);
        motorFrontRight.setVelocity(-velocity);
        motorBackRight.setVelocity(-velocity);
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
        double endt = (getRuntime() + addtime);
        while (t < endt) {
            telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
            telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
            telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());
            t = getRuntime();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setVelocity(0);
        motorBackLeft.setVelocity(0);
        motorFrontRight.setVelocity(0);
        motorBackRight.setVelocity(0);

    }

    public void driveXdir(double distance, double power) {

        double sign = distance/Math.abs(distance);
        distance = Math.abs(distance)/0.5;
        double rpm = power * 500;
        double speed = ((rpm*3*Math.PI)/60);
        double inverse = 1/speed;
        double addtime = distance * inverse;
        //double StrafeRotations = 30/circumference;
        //int StrafeDrivingTarget =  (int)(StrafeRotations*MOTOR_TICK_COUNTS);
        double startt = getRuntime();
        double t = getRuntime();

        power = sign *power;
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(-power);

        double endt = (getRuntime() + addtime);
        while (t < endt) {
            telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
            telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
            telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());
            t = getRuntime();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public void driveArm(int angle, double power) {
        double turnco = 0;
        double pos = 0;
        dist1 = distl;
        dist2 = dist1;
        dist3 = dist2;
        dist1 = distance.getDistance(DistanceUnit.INCH);
        dist = ((dist1 + dist2+dist3)/3);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        previousError = 0;
        integral = 0;
        double target = 3.25*Math.tan((angle-90)*(Math.PI/180))+10.5;
        double error = target - dist;
        double b = getRuntime();
        double dt = 0.001;
        while(Math.abs(target - dist) > 0.05){
            dist3 = dist2;
            dist2 = dist1;
            dist1 = distance.getDistance(DistanceUnit.INCH);
            dist = ((dist1 + dist2 + dist3)/3);
            previousError = error;
            b = getRuntime();
            error = target - dist;
            integral = integral + error * dt;
            power = (kP*error + kI*integral);
            double denom = Math.max(Math.abs(power),1);
            power = power/denom;
            Arm.setPower(power);
            dt = getRuntime()-b;




        }


        Arm.setPower(0);



    }

    public void driveAlien(float distance, double power) {


        //double StrafeRotations = 30/circumference;
        //int StrafeDrivingTarget =  (int)(StrafeRotations*MOTOR_TICK_COUNTS);
        double rotationsNeeded = distance/360.0;
        int encoderDrivingTarget = (int)(distance*625);
        int target = encoderDrivingTarget;




        Alien.setTargetPosition(target);

        Alien.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Alien.setPower(power);


        while (Alien.isBusy()) {

        }
        Alien.setPower(0);


    }
    public void closeClaw(){
        Claw.setPosition(1);
        double t = getRuntime();
        while(getRuntime() < (t+1.5))
        {}
    }
    public void openClaw(){
        Claw.setPosition(0);
        double t = getRuntime();
        while(getRuntime() < (t+1.5))
        {}
    }
}