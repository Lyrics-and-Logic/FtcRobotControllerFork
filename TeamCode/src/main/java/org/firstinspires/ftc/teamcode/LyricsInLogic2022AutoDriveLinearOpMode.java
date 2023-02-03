/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;




abstract public class LyricsInLogic2022AutoDriveLinearOpMode extends LinearOpMode {

    // Declare OpMode members.
    final protected ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LR;
    private DcMotor RR;

    public LyricsInLogic2022AutoDriveLinearOpMode() {

    }

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // Our motor's web site is https://www.andymark.com/products/neverest-orbital-20-gearmotor
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6; // Ticks per revolution from
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.779528 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.05;
    static final double     TURN_SPEED              = 0.3;
    static final int        NINETY_DEGREES       = 34;

    private int degreesToInches(double degrees){
        int magnitude = degrees < 0 ? -1 : 1;
        double factor = Math.abs((int) degrees) / 90.0;
        int inches = (int) Math.ceil(factor * NINETY_DEGREES);
        return magnitude * inches;
    }

    protected void currentPosition(DcMotor leftWheel, DcMotor rightWheel) {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Position ", "%7d :%7d",
                leftWheel.getCurrentPosition(),
                rightWheel.getCurrentPosition());
        telemetry.update();

    }

    protected void currentPosition() {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("%s", "LF=" + LF.getCurrentPosition() +
                " RF=" + RF.getCurrentPosition() +
                " LR=" + LR.getCurrentPosition() +
                " RR=" + RR.getCurrentPosition());
        telemetry.update();
    }

    protected void initHardwareMap(){
        LF = hardwareMap.get(DcMotor.class, "RR");
        LR = hardwareMap.get(DcMotor.class, "RF");
        RF = hardwareMap.get(DcMotor.class, "LR");
        RR = hardwareMap.get(DcMotor.class, "LF");

//        LF.setDirection(DcMotorSimple.Direction.REVERSE);
//        LR.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoder();
    }

    protected int getPrintMessageDelay(){
        return 750;
    }

    protected void moveForward(int distance){
        logMessage("move forward");
        moveFourWheels(LF, 1, RF, 1,
                LR, 1, RR, 1,
                DRIVE_SPEED, distance);
    }

    protected void moveBackward(int distance){
        logMessage("move backward");
        moveFourWheels(LF, -1, RF, -1,
                LR, -1, RR, -1,
                DRIVE_SPEED, -distance);
    }

    protected void moveLeft(int distance){
        logMessage("move left");
        moveFourWheels(LF, -1, RF, 1,
                LR, 1, RR, -1,
                DRIVE_SPEED, distance);
    }

    protected void moveRight(int distance){
        logMessage("move left");
        moveFourWheels(LF, 1, RF, -1,
                LR, -1, RR, 1,
                DRIVE_SPEED, distance);
    }

    protected void leftForwardDiag(int distance){
        logMessage("move left forward diag");
        moveFourWheels(LF, 0, RF, 1,
                LR, 1, RR, 0,
                DRIVE_SPEED, distance);
    }

    protected void rightForwardDiag(int distance){
        logMessage("move right forward diag");
        moveFourWheels(LF, 1, RF, 0,
                LR, 0, RR, 1,
                DRIVE_SPEED, distance);
    }

    protected void leftBackwardDiag(int distance){
        logMessage("move left backward diag");
        moveFourWheels(LF, -1, RF, 0,
                LR, 0, RR, -1,
                DRIVE_SPEED, distance);
    }

    protected void rightBackwardDiag(int distance){
        logMessage("move left backward diag");
        moveFourWheels(LF, 0, RF, -1,
                LR, -1, RR, 0,
                DRIVE_SPEED, distance);
    }

    protected void turnLeft(double degrees){
        int distance = degreesToInches(degrees);
        logMessage("turning left");
        moveFourWheels(LF, -1, RF, 1,
                LR, -1, RR, 1,
                DRIVE_SPEED, distance);
    }

    protected void turnRight(double degrees){
        int distance = degreesToInches(degrees);
        logMessage("turning right");
        moveFourWheels(LF, 1, RF, -1,
                LR, 1, RR, -1,
                DRIVE_SPEED, distance);
    }

    protected void logMessage(String msg){
        printMessage(msg, getPrintMessageDelay());
    }
    protected void printMessage(String msg, int n){
        telemetry.addData("", "%s", msg);
        telemetry.update();
        sleep(n);

    }

    protected void setTargetPosition(DcMotor wheel, double distance, int direction){
        if (direction == 0)
            return;
        int targetPosition = wheel.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH * direction);
        wheel.setTargetPosition(targetPosition);
    }

    protected void moveFourWheels(DcMotor leftFrontWheel,
                                  int leftFrontDirection,
                                  DcMotor rightFrontWheel,
                                  int rightFrontDirection,
                                  DcMotor leftRearWheel,
                                  int leftRearDirection,
                                  DcMotor rightRearWheel,
                                  int rightRearDirection,
                                  double power,
                                  double distance){
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            setTargetPosition(leftFrontWheel, distance, leftFrontDirection);
            setTargetPosition(rightFrontWheel, distance, rightFrontDirection);
            setTargetPosition(leftRearWheel, distance, leftRearDirection);
            setTargetPosition(rightRearWheel, distance, rightRearDirection);

            currentPosition();

            // Turn On RUN_TO_POSITION
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontWheel.setPower(power * leftFrontDirection != 0 ? 1 : 0);
            rightFrontWheel.setPower(power * rightFrontDirection != 0 ? 1 : 0);
            leftRearWheel.setPower(power * leftRearDirection != 0 ? 1 : 0);
            rightRearWheel.setPower(power * rightRearDirection != 0 ? 1 : 0);

            int i = 0;
            while (opModeIsActive() &&
                    //(runtime.seconds() < 5.0) &&
                    reachedTarget(leftFrontWheel)){
                // Display it for the driver.
                currentPosition();
            }

            // Stop all motion;
            stopMotion();
            currentPosition();

            // Turn off RUN_TO_POSITION
            turnOff();
            sleep(250);   // optional pause after each move.
            logMessage("Done");
        }
    }

    protected void turnOff() {
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected boolean reachedTarget(DcMotor wheel){
        return wheel.isBusy();
    }

    protected void stopMotion() {
        LR.setPower(0);
        RR.setPower(0);
        LF.setPower(0);
        RF.setPower(0);
    }

    protected void resetEncoder() {
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

