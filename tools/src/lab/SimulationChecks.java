package lab;

import java.lang.reflect.Field;
import java.util.concurrent.FutureTask;
import java.util.concurrent.TimeUnit;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.learning.*;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.controller.VirtualBot;

/** Runs the real JavaFX controller/OpMode loop and dyn4j world, with no visible window. */
public class SimulationChecks {
    private static VirtualRobotController controller;
    private static Object field(Object object, String name) throws Exception {
        Class<?> type = object.getClass();
        while (type != null) {
            try { Field f = type.getDeclaredField(name); f.setAccessible(true); return f.get(object); }
            catch (NoSuchFieldException e) { type = type.getSuperclass(); }
        }
        throw new NoSuchFieldException(name);
    }
    private static void fx(Runnable work) throws Exception {
        FutureTask<Void> task = new FutureTask<>(work, null);
        Platform.runLater(task);
        task.get(10, TimeUnit.SECONDS);
    }
    private static Button button() throws Exception { return (Button)field(controller, "driverButton"); }
    @SuppressWarnings("unchecked")
    private static void choose(Class<?> type) throws Exception {
        fx(() -> { try { ((ComboBox<Class<?>>)field(controller,"cbxOpModes")).getSelectionModel().select(type); }
            catch(Exception e){ throw new RuntimeException(e); } });
    }
    private static void click() throws Exception { fx(() -> { try { button().fire(); } catch(Exception e){throw new RuntimeException(e);} }); }
    private static void require(boolean value, String message) { if (!value) throw new AssertionError(message); }
    private static void runLearning(Class<?> type, double x, double y, double heading) throws Exception {
        choose(type); click(); Thread.sleep(300); click();
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(15);
        Object opmode = field(controller,"opMode");
        boolean completed = false;
        while (System.nanoTime() < deadline) {
            Thread.sleep(100);
            if ((boolean)field(opmode,"finished")) { completed = true; break; }
        }
        Follower follower = (Follower)field(opmode,"follower");
        Pose pose = follower.pose();
        String result = (String)field(opmode,"result");
        click();
        double angleError = Math.atan2(Math.sin(pose.heading()-Math.toRadians(heading)), Math.cos(pose.heading()-Math.toRadians(heading)));
        require(completed && result.equals("Reached target"), type.getSimpleName()+": "+result+" pose="+pose);
        require(Math.hypot(pose.x()-x,pose.y()-y)<1.5 && Math.abs(angleError)<Math.toRadians(5), "Endpoint mismatch: "+pose);
        System.out.printf("PASS %s: x=%.3f y=%.3f heading=%.2f%n",type.getSimpleName(),pose.x(),pose.y(),Math.toDegrees(pose.heading()));
    }
    public static void main(String[] args) {
        int exitCode = 1;
        try {
            Platform.startup(() -> {});
            fx(() -> { try {
                FXMLLoader loader = new FXMLLoader(VirtualRobotController.class.getResource("virtual_robot.fxml"));
                loader.load(); controller = loader.getController(); controller.setConfig(null);
            } catch(Exception e){ throw new RuntimeException(e); } });
            VirtualBot bot = (VirtualBot)field(controller,"bot");
            require(bot.getClass().getSimpleName().equals("MecDynamicBot"), "Wrong robot model");
            bot.getHardwareMap().setActive(true);
            GoBildaPinpointDriver pinpoint = bot.getHardwareMap().get(GoBildaPinpointDriver.class,"pinpoint");
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,10,20,AngleUnit.DEGREES,0));
            pinpoint.setHeading(90,AngleUnit.DEGREES); pinpoint.update();
            Pose2D reset = pinpoint.getPosition();
            require(Math.abs(reset.getHeading(AngleUnit.DEGREES)-90)<0.01 && Math.abs(reset.getX(DistanceUnit.INCH)-10)<0.01, "Pinpoint heading reset failed");
            bot.getHardwareMap().setActive(false);
            System.out.println("PASS Pinpoint heading reset preserves position");
            runLearning(LearnForward.class,72,84,90);
            runLearning(LearnLeft.class,60,72,90);
            runLearning(LearnTurn.class,72,72,180);
            // Run the upstream example with Ivy's real scheduler, then exercise STOP mid-path.
            choose(org.firstinspires.ftc.teamcode.pedropathing.examples.IvyOpMode.class);
            click(); Thread.sleep(300); click(); Thread.sleep(2000);
            Object opmode = field(controller,"opMode");
            Follower follower = (Follower)field(opmode,"follower");
            Pose pose = follower.pose();
            require(Math.hypot(pose.x()-48,pose.y()-48)>2,"Upstream Ivy example did not move");
            DcMotor[] motors = new DcMotor[4];
            String[] names = {"front_left_motor","front_right_motor","back_left_motor","back_right_motor"};
            for(int i=0;i<4;i++) motors[i]=bot.getHardwareMap().get(DcMotor.class,names[i]);
            click(); Thread.sleep(150);
            for(DcMotor motor:motors) require(motor.getPower()==0,"Motor nonzero after STOP");
            System.out.println("PASS upstream Ivy example movement and STOP: "+pose);
            System.out.println("ALL SIMULATION CHECKS PASSED");
            exitCode = 0;
        } catch(Throwable t) { t.printStackTrace(); }
        finally { Platform.exit(); System.exit(exitCode); }
    }
}
