//   ******Front******
//   * 0           2 *
//   *               *
//   *               *
//   *               *
//   * 1           3 *
//   ******Back*******

// Motor 0 --- Pin 0 --- motorFrontLeft
// Motor 1 --- Pin 1 --- motorBackLeft
// Motor 2 --- Pin 2 --- motorFrontRight
// Motor 3 --- Pin 3 --- motorBackRight

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous

public class AutoManTest extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor Arm = null;
    private DcMotorEx Alien = null;
    SparkFunOTOS myOtos;
    private DcMotor pickup = null;
    private DcMotor wheel = null;
    private Servo Claw = null;

    public double ki = 0.45;

    public double kd = 0.05;
    SparkFunOTOS.Pose2D pos;


    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
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






    boolean toggleB = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 3; // Left
        usbFacingDirectionPosition = 0; // Up

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;




        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        Arm = hardwareMap.dcMotor.get("Arm");
        Alien = (DcMotorEx) hardwareMap.dcMotor.get("Alien");
        Claw = hardwareMap.get(Servo.class, "Claw");
        wheel = hardwareMap.dcMotor.get("wheel");
        pickup = hardwareMap.dcMotor.get("pickup");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        configureOtos();
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
        pos = myOtos.getPosition();
        //DrivetoCoords(10,0,90);
        /*driveYdir(-30);
        turntoangle(0);
        shoot(-1300);
        driveYdir(-5);
//        double t = getRuntime() + 8;
//        while(getRuntime() < t){}
        pos = myOtos.getPosition();
        driveYdir(-35/1.2-pos.y);
        turntoangle(40);
        driveXdir(-15);
        turntoangle(40);
        Arm.setPower(1);
        wheel.setPower(1);
        driveYdir(20);
        Arm.setPower(0);
        wheel.setPower(0);*/


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry


        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double botHeading = orientation.getYaw(AngleUnit.RADIANS);





        pos = myOtos.getPosition();
        double xpow = gamepad1.left_stick_x;
        double ypow = -gamepad1.left_stick_y;
        double hpow = gamepad1.right_stick_x;
        double rotX = xpow * Math.cos(Math.toRadians(-pos.h)) - ypow * Math.sin(Math.toRadians(-pos.h));
        rotX = rotX*1.1;
        double rotY = xpow * Math.sin(Math.toRadians(-pos.h)) + ypow * Math.cos(Math.toRadians(-pos.h));
        double denominator = Math.max(Math.abs(ypow) + Math.abs(xpow) + Math.abs(hpow), 1);
        motorFrontLeft.setPower((rotY + rotX - hpow)/denominator);
        motorBackLeft.setPower((rotY - rotX - hpow)/denominator);
        motorFrontRight.setPower((rotY - rotX + hpow)/denominator);
        motorBackRight.setPower((rotY + rotX + hpow)/denominator);
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);

        // Update the telemetry on the driver station

        telemetry.addData("Alien", Alien.getCurrentPosition());
        telemetry.addData("Arm", Arm.getCurrentPosition());
        telemetry.addData("Right Front", motorFrontRight.getCurrentPosition());
        telemetry.addData("Left Front", motorFrontLeft.getCurrentPosition());
        telemetry.addData("Right Back", motorBackRight.getCurrentPosition());
        telemetry.addData("Left Back", motorBackLeft.getCurrentPosition());

        telemetry.addData("A", toggleB);
        telemetry.addData("servo",Claw.getPosition());






        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Yaw (Z)", "%.2f Rad. (Heading)", orientation.getYaw(AngleUnit.DEGREES));





        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }



    @Override
    public void stop() {

    }
    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.2);
        myOtos.setAngularScalar(1.0035);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    private void driveYdir(double dist){
        dist = dist/1.2;
        pos = myOtos.getPosition();
        double target = pos.y + dist;
        double epsilon = 0.3;
        double derivative;

        double t = getRuntime() - 1;
        double error = (target-pos.y);
        double preverror = error;
        while(Math.abs(error)>epsilon){

            pos = myOtos.getPosition();
            error = (target-pos.y);
            derivative = (error-preverror)/(getRuntime()-t);
            t = getRuntime();
            double motorpower = Math.max(-1,Math.min(1,(ki*error+kd*derivative)));
            motorFrontLeft.setPower(motorpower);
            motorBackLeft.setPower(motorpower);
            motorFrontRight.setPower(motorpower);
            motorBackRight.setPower(motorpower);

            preverror = error;
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

    }
    private void driveXdir(double dist){
        dist = dist/1.1;
        pos = myOtos.getPosition();
        double target = pos.x + dist;
        double epsilon = 0.5;
        double derivative;

        double t = getRuntime() - 1;
        double error = (target-pos.x);
        double preverror = error;
        while(Math.abs(error)>epsilon){

            pos = myOtos.getPosition();
            error = (target-pos.x);
            derivative = (error-preverror)/(getRuntime()-t);
            t = getRuntime();
            double motorpower = Math.max(-1,Math.min(1,(0.45*error+0.05*derivative)));
            motorFrontLeft.setPower(motorpower);
            motorBackLeft.setPower(-motorpower);
            motorFrontRight.setPower(-motorpower);
            motorBackRight.setPower(motorpower);
            preverror = error;
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

    }
    private void turntoangle(double ang){
        //dist = dist/1.1;
        pos = myOtos.getPosition();
        double botheading = pos.h;
        double target = ang;
        double epsilon = 0.3;
        double derivative;

        double t = getRuntime() - 1;
        double error = (target-botheading);
        double preverror = error;
        while(Math.abs(error)>epsilon){

            pos = myOtos.getPosition();
            botheading = pos.h;
            error = (target-botheading);
            derivative = (error-preverror)/(getRuntime()-t);
            t = getRuntime();
            double motorpower = Math.max(-1,Math.min(1,(0.30*error+0.01*derivative)));
            motorFrontLeft.setPower(-motorpower);
            motorBackLeft.setPower(-motorpower);
            motorFrontRight.setPower(motorpower);
            motorBackRight.setPower(motorpower);
            preverror = error;
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            //telemetry.addData("Heading angle", pos.h);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)",pos.h);
            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);


    }
    private void shoot(double power){
        Alien.setVelocity(power);
        double t = getRuntime() + 3;
        while(getRuntime() < t){
            telemetry.addData("speed",Alien.getVelocity());
            telemetry.update();
        }
        Arm.setPower(1);
        wheel.setPower(-1);
        t = getRuntime() + 1;
        while(getRuntime() < t){}
        Arm.setPower(0);
        wheel.setPower(0);
        t = getRuntime() + 1;
        while(getRuntime() < t){}
        Arm.setPower(1);
        wheel.setPower(-1);
        t = getRuntime() + 1;
        while(getRuntime() < t){}
        Arm.setPower(0);
        wheel.setPower(0);
        t = getRuntime() + 1;
        while(getRuntime() < t){}
        Arm.setPower(1);
        wheel.setPower(-1);
        t = getRuntime() + 2;
        while(getRuntime() < t){}
        Arm.setPower(0);
        wheel.setPower(0);
        Alien.setPower(0);
    }
    private void DrivetoCoords(double x, double y, double h){
        x = x/1.2;
        y = y/1.2;
        pos = myOtos.getPosition();
        double ytarget = y;
        double xtarget = x;
        double htarget = h;
        double epsilon = 0.1;
        double xderivative;
        double yderivative;
        double hderivative;
        double t = getRuntime() - 1;
        double yerror = (ytarget-pos.y);
        double xerror = (xtarget-pos.x);
        double herror = (htarget-pos.h);
        double prevxerror = xerror;
        double prevyerror = yerror;
        double prevherror = herror;
        while(Math.abs(xerror)>epsilon || Math.abs(yerror)>epsilon || Math.abs(herror)>epsilon){

            pos = myOtos.getPosition();
            yerror = (ytarget-pos.y);
            xerror = (xtarget-pos.x);
            herror = (htarget-pos.h);
            xderivative = (xerror-prevxerror)/(getRuntime()-t);
            yderivative = (yerror-prevyerror)/(getRuntime()-t);
            hderivative = (herror-prevherror)/(getRuntime()-t);
            t = getRuntime();
            double xpow = 0.45*xerror+0.05*xderivative;
            double ypow = ki*yerror+kd*yderivative;
            double hpow = 0.3*herror+0.01*hderivative;

            double rotX = xpow * Math.cos(Math.toRadians(-pos.h)) - ypow * Math.sin(Math.toRadians(-pos.h));
            double rotY = xpow * Math.sin(Math.toRadians(-pos.h)) + ypow * Math.cos(Math.toRadians(-pos.h));
            double denominator = Math.max(Math.abs(ypow) + Math.abs(xpow) + Math.abs(hpow), 1);
            motorFrontLeft.setPower((rotY + rotX - hpow)/denominator);
            motorBackLeft.setPower((rotY - rotX - hpow)/denominator);
            motorFrontRight.setPower((rotY - rotX + hpow)/denominator);
            motorBackRight.setPower((rotY + rotX + hpow)/denominator);

            prevxerror = xerror;
            prevyerror = yerror;
            prevherror = herror;
            
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}
