import lejos.hardware.motor.NXTMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.EncoderMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PR {
	//public static BasicMovement bm = new BasicMovement();
	public static ForwardKinematics fk = new ForwardKinematics();
	public static InverseKinematics ik = new InverseKinematics();
	public static Pathfinding pf = new Pathfinding();
	
	static EV3LargeRegulatedMotor firstM = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor secondM = new EV3LargeRegulatedMotor(MotorPort.C);
	static EncoderMotor thirdM = new NXTMotor(MotorPort.D);
	static EV3TouchSensor ts = new EV3TouchSensor(SensorPort.S1);

	
	public static void main(String[] args) {

		fk.run(firstM, secondM, ts);
//		ik.run(firstM, secondM, ts);
//		pf.run(firstM, secondM, thirdM, ts);

	}
}