import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import java.util.*
import kotlin.collections.ArrayList

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(55.0, 40.0, 0.0, 180.0.toRadians, 180.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val startPose = Pose2d(-63.5, -34.0, 0.0.toRadians)
    private val shootPose = Pose2d(-63.5, -18.0, Math.toRadians(0.0))
    private val pose2 = Pose2d(-3.0,-21.0, 0.0.toRadians)
    private val shootend = Pose2d(-3.0,-38.0, 0.0.toRadians)
    private val dropend = Pose2d(63.0,-47.0, 0.0.toRadians)
    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val strafeLeftTraj = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints);

        val builder1 = TrajectoryBuilder(shootPose, shootPose.heading, combinedConstraints)

        val toShooterPos = TrajectoryBuilder(pose2,pose2.heading, combinedConstraints)

        val dropSite = TrajectoryBuilder(shootend, shootend.heading, combinedConstraints)

        val park = TrajectoryBuilder(dropend, dropend.heading, combinedConstraints)


        //shoot rings
        strafeLeftTraj.lineToConstantHeading(Vector2d(-63.5, -18.0))
        builder1.lineToConstantHeading(Vector2d(-3.0,-21.0))
        toShooterPos.lineToConstantHeading(Vector2d(-3.0,-38.0))

        //drop A site
        //dropSite.splineToConstantHeading(Vector2d(10.0,-47.0), 0.0.toRadians)

        //drop B site
        //dropSite.splineToConstantHeading(Vector2d(38.0,-21.0), 0.0.toRadians)

        //drop B site
        dropSite.splineToConstantHeading(Vector2d(63.0,-47.0), 0.0.toRadians)

        park.lineToLinearHeading(Pose2d(7.0,0.0, Math.toRadians(180.0)))



        list.add(strafeLeftTraj.build())
        list.add(builder1.build())
        list.add(toShooterPos.build())
        list.add(dropSite.build())
        list.add(park.build())

        return list
    }

    fun drawOffbounds() {
        //GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
