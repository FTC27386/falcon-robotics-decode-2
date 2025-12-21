package org.firstinspires.ftc.teamcode.Mechanisms;


import static org.firstinspires.ftc.teamcode.opMode.teleOp.flywheel_speed;
import static org.firstinspires.ftc.teamcode.opMode.teleOp.hood_pos;
import static java.lang.Math.sqrt;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utility.RobotConstants;
import org.firstinspires.ftc.teamcode.Utility.UtilMethods;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class drivetrainSystem extends SubsystemBase {
    public static Pose
            currentPose = new Pose(0, 0, Math.toRadians(90)),
            startPose,
            targ;
    public Follower follower;
    public Vector vel;
    public double
            x,
            distanceX,
            y,
            distanceY,
            dist,
            vx,
            vy,
            heading,
            unnormalizedHeading,
            field_angle,
            zoneBuffer = 7.5 * Math.sqrt(2);
    public Supplier<Pose> poseSupplier = this::getCurrentPose;
    boolean robotCentricDrive = false;

    public drivetrainSystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(RobotConstants.autoEndPose == null ? new Pose(8, 8, Math.toRadians(90)) : RobotConstants.autoEndPose);
        follower.update();
        if (RobotConstants.current_color == null || RobotConstants.current_color == RobotConstants.ALLIANCE_COLOR.BLUE) {
            targ = new Pose(0, 144);
        } else {
            targ = new Pose(144, 144);
        }
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        x = currentPose.getX();
        y = currentPose.getY();
        heading = currentPose.getHeading();
        distanceX = targ.getX() - x;
        distanceY = targ.getY() - y;
        unnormalizedHeading = follower.getTotalHeading();
    }

    public double yoCalcDist() {
        return Math.hypot(distanceX, distanceY);
    }

    public double yoCalcAim()  //calculate adjusted turret angle in degrees
    {
        field_angle = (90 - Math.toDegrees(Math.atan2(distanceY, distanceX)));
        // return Math.toDegrees((heading)-Math.PI/2) + field_angle;
        return -UtilMethods.AngleDifference(Math.toDegrees(unnormalizedHeading), 0) + field_angle;

        //equivalent: Math.toDegrees(currentpose.getHeading() - Math.PI/2)

        //(90 - Math.toDegrees(Math.atan2(distanceY,distanceX)))
    }

    public double yoCalcHood() {
        /*
        dist = yoCalcDist();
        if (dist >= 50.2778 && dist <= 89.5204) {
            return (0.00000106851 * Math.pow(dist, 4))
                    - (0.000298793 * Math.pow(dist, 3))
                    + (0.0307389 * Math.pow(dist, 2))
                    - (1.37843 * dist)
                    + 23.22303;
        } else if (dist > 89.5204 && dist <= 110.2215) {
            return (-0.0000318774 * Math.pow(dist, 3))
                    + (0.00925592 * Math.pow(dist, 2))
                    - (0.876897 * dist)
                    + 27.89043;
        } else {
            return 0.5;
        }
         */
        return hood_pos;
    }

    public double yoCalcSpeed() {
        /*
        dist = yoCalcDist();
        if (dist >= 50.2778 && dist <= 89.5204) {
            return -1700;
        } else if (dist > 89.5204 && dist <= 110.2215) {
            return -2000;
        } else {
            return 0;
        }
         */
        return flywheel_speed;
    }

    public void teleOpDrive(double axial, double lateral, double yaw) {

        follower.setTeleOpDrive(
                -axial,
                -lateral,
                -yaw,
                true);
    }

    public Vector prediction() {
        vel = follower.getVelocity();
        vx = vel.getXComponent();
        vy = vel.getYComponent();

        // 1. Calculate the 'Approach Velocity' (Closing speed toward goal)
        // This is the Y-component in a goal-centric frame
        double relVelY = (vx * distanceX + vy * distanceY) / dist;

        // 2. Calculate the 'Lateral Velocity' (Sideways movement relative to goal)
        // This is the X-component in a goal-centric frame
        double relVelX = (vy * distanceX - vx * distanceY) / dist;

        return new Vector(relVelX, relVelY);
    }

    public Pose getPredictedPose(double secondsInFuture) {
        // 1. Get current state
        Pose currentPose = follower.getPose();
        Vector currentVel = follower.getVelocity(); // Field-centric velocity

        // 2. Calculate displacement (Velocity * Time)
        double deltaX = currentVel.getXComponent() * secondsInFuture;
        double deltaY = currentVel.getYComponent() * secondsInFuture;

        // 3. Return the predicted position
        return new Pose(
                currentPose.getX() + deltaX,
                currentPose.getY() + deltaY,
                currentPose.getHeading() // Note: Heading might also change!
        );
    }

    public void reloc(Pose reloc) {
        follower.setPose(reloc);
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public Pose getTarg() {
        return targ;
    }

    public void relocTarget(Pose reloc) {
        targ = reloc;
    }

    public boolean inZone() {
        return (y > Math.abs(x - 72) + 72 - zoneBuffer) || (y < -Math.abs(x - 72) + 24 + zoneBuffer);
    }
}
