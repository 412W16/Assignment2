import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import javax.imageio.ImageIO;

import java.awt.Color;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;


public class Pathfinding {

	public static double l1 = 12.25;
	public static double l2 = 10.2;
	public static double l3 = 7;

	public static EV3LargeRegulatedMotor firstM;
	public static EV3LargeRegulatedMotor secondM;
	public static EncoderMotor thirdM;
	public static EV3TouchSensor ts;
	
	public void run(EV3LargeRegulatedMotor f, EV3LargeRegulatedMotor s, EncoderMotor tm, EV3TouchSensor t) {
		// TODO Auto-generated method stub
		firstM = f;
		secondM = s;
		thirdM = tm;
		ts = t;
		
		double[] point = {0, 25, 5};
//		analytical3D(point, true);
//		Q9();
		Q13();
//		C8();
		Button.waitForAnyPress();
	}

	

	private void A8() {
		// TODO Auto-generated method stub
		// Draw a line defined by two points
		double[] pos1 = {-5, 10};
		double[] pos2 = {10, 15};
		
		
		Move(pos1, pos2);
		
	}
	
	private void B8() {
		double[] pos1 = {-5, 10};
		double theta = 1.349;
		double dist = 10;
		
		double[] pos2 = {pos1[0] + dist * Math.cos(theta), pos1[1] + dist*Math.sin(theta)};
		Move(pos1, pos2);
	}
	
	private void C8() {
		int numPoints = 5;
		double[][] points = { {-5, 10}, {-2.5, 13}, {0, 15}, {-2.5, 17}, {-5, 19} };
		for (int i = 0; i < numPoints-1; i++) {
			Move(points[i], points[i+1]);
		}
	}
	
	private void Q9() {
		int numPoints = 20;
		double[][] points = new double[numPoints][2];
		// Start position
		points[0][0] = 9.5;
		points[0][1] = 6.3;
		// Point 2, straight up
		points[1][0] = points[0][0];
		points[1][1] = points[0][1] + 4.2;
		// Point 3, diagonally to the left and up
		points[2][0] = points[1][0] - 2.1;
		points[2][1] = points[1][1] + 1.9;
		// Point 4, horizontal to the left
		points[3][0] = points[2][0] - 1.8;
		points[3][1] = points[2][1];
		// Point 5, midpoint between half circle
		points[4][0] = points[3][0] + 1.7;
		points[4][1] = points[3][1] + 2.7;
		// Point 6, top of half circle
		points[5][0] = points[4][0] - 1.7;
		points[5][1] = points[4][1] + 2.8;
		// Point 7, horizontal to the left
		points[6][0] = points[5][0] - 1.8;
		points[6][1] = points[5][1];
		// Point 8, Vertical down
		points[7][0] = points[6][0];
		points[7][1] = points[6][1] - 3.6;
		// Point 9 horizontal to the left
		points[8][0] = points[7][0] - 1.8;
		points[8][1] = points[7][1];
		// Point 10 vertical up
		points[9][0] = points[8][0];
		points[9][1] = points[8][1] + 3.6;
		//Point 11 horiztonal to the left
		points[10][0] = points[9][0] - 1.8;
		points[10][1] = points[9][1];
		// Point 12 vertical down
		points[11][0] = points[10][0];
		points[11][1] = points[10][1] - 3.6;
		// Point 13 horizontal to the left
		points[12][0] = points[11][0] - 1.8;
		points[12][1] = points[11][1];
		// Point 14 vertical up
		points[13][0] = points[12][0];
		points[13][1] = points[12][1] + 3.6;
		// Point 15 horizontal to the left
		points[14][0] = points[13][0] - 2.4;
		points[14][1] = points[13][1];
		// Point 16 first point on arcMath.sqrt(Math.pow(point[1], 2) + Math.pow(point[2],  2));
		points[15][0] = points[14][0] + 0.4;
		points[15][1] = points[14][1] - 1.25;
		// Point 17 second point on arc
		points[16][0] = points[15][0] - 0.85;
		points[16][1] = points[15][1] - 2.39;
		// Point 18 last point on arc
		points[17][0] = points[16][0] - 2.15;
		points[17][1] = points[16][1] - 2.1;
		// Point 19 horizontal to the left
		points[18][0] = points[17][0] - 3.0;
		points[18][1] = points[17][1];
		// Point 20 vertical down
		points[19][0] = points[18][0];
		points[19][1] = points[18][1] - 4.7;
		
		analytical(points[0], true);

		for (int i = 0; i < numPoints-1; i++) {
			Move(points[i], points[i+1]);
			Delay.msDelay(1000);
		}
		
	}
	
	private void Q13() {
		BufferedImage img = null;
		try {
		    img = ImageIO.read(Pathfinding.class.getResource("/circle.png"));
		} catch (IOException e) {
			System.out.println("IMAGE NOT FOUND");
			return;
		}
		System.out.println("IMAGE READ SUCCESSFUL");
		int w = img.getWidth();
		int h = img.getHeight();
		int maxPoints = w*h;
		int totalPoints = 0;
		double[][] points = new double[maxPoints][2];
		
		for (int i = 0; i < h; i++) {
			for (int j = 0; j < w; j++) {
				Color color = new Color(img.getRGB(j, i));
//				System.out.println(color.toString());
				if (color != Color.white) {
//					System.out.format("w: %d, h: %d\n", i, j);
					points[totalPoints][0] = i;
					points[totalPoints][1] = j;
					totalPoints++;
				}
			}
		}
		
		for (int i = 0; i < totalPoints; i++) {
			// x/w = x'/10
			// y/h = y'/10
			points[i][0] = points[i][0] * 15 / w;
			points[i][1] = points[i][1] * 15 / h;
		}
		
		analytical(points[0], true);
		
		for (int i = 0; i < totalPoints - 1; i++) {
			Move(points[i], points[i+1]);
		}
		
	}
	
	private void Move(double[] pos1, double[] pos2) {
		int size = 20;
		double deltaX = (pos2[0] - pos1[0]);
		double deltaY = (pos2[1] - pos1[1]);
		double incrementX = deltaX/size;
		double incrementY = deltaY/size;
		double[][] points = new double[size+1][2];
		
		int i = 0;
		
		double x = pos1[0];
		double y = pos1[1];
		for(i = 0; i < size; i++) {
			points[i][0] = x;
			points[i][1] = y;
			x += incrementX;
			y += incrementY;
		}
		points[size][0] = pos2[0];
		points[size][1] = pos2[1];
		
		double[] current = new double[2];
		for(i=0;i<=size;i++) {
			current[0] = points[i][0];
			current[1] = points[i][1];
			analytical(current, true);
		}
		System.out.format("x: %f y: %f \n", current[0], current[1]);
	}

	public void moveMotor(EV3LargeRegulatedMotor m, double theta) {
		m.setSpeed(30);
		m.rotateTo((int)theta);
		m.stop();
	}
	
	public void moveMotor(EncoderMotor m, double theta) {
		m.setPower(20);
		double initial = m.getTachoCount();

		m.forward();
		while(m.getTachoCount()-initial < theta){
			continue;
		}
		m.stop();
	}
	
	public void analytical(double[] pos, boolean interior) {
		double theta2=0;
		double theta1=0;
		
		double x = pos[0];
		double y = pos[1];
		
		double num=0;
		
		double D = (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(l1, 2) - Math.pow(l2, 2))/(2 * l1 * l2);
		
		
		num = Math.sqrt(1-Math.pow(D, 2));
		
		if(interior) {
			theta2 = Math.atan2(num, D);
		} else {
			theta2 = Math.atan2(-num, D);
		}
		
		theta1 = Math.atan2(y,x) - Math.atan2(l2*Math.sin(theta2), l1 + l2 * Math.cos(theta2));
		
//		System.out.format("Angle1: %f Angle2: %f\n", -theta1, theta2);
		theta1 = -(theta1*(180/Math.PI));
		theta2 = theta2*(180/Math.PI);
//		System.out.format("Angle1: %f Angle2: %f\n", -theta1, theta2);
		moveMotor(firstM, theta1);
		moveMotor(secondM, theta2);
	}
	
	public void analytical3D(double[] pos, boolean interior) {
		double theta3 = 0;
		double theta2 = 0;
		double theta1 = 0;
		
		double ll2 = l2 + 3.5;
		
		double x = pos[0];
		double y = pos[1];
		double z = pos[2];
		
		double l3_p = Math.sqrt(Math.pow(l3, 2) - Math.pow(z, 2));
		
		theta3 = Math.asin(z/l3);
		
		double D = (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(l1, 2) - Math.pow(ll2 + l3_p, 2)) / (2 * l1 * (ll2 + l3_p));
		
		if(interior) {
			theta2 = Math.atan2(Math.sqrt(1 - Math.pow(D, 2)), D);
		} else {
			theta2 = Math.atan2(-Math.sqrt(1 - Math.pow(D, 2)), D);
		}
		
		theta1 = Math.atan2(y, x) - Math.atan2(ll2+l3_p, l1 + (ll2+l3_p)*Math.cos(theta2));
		theta3 = theta3 * (180/Math.PI);
		theta2 = theta2 * (180/Math.PI);
		theta1 = -theta1 * (180/Math.PI);
		moveMotor(firstM, theta1);
		moveMotor(secondM, theta2);
		moveMotor(thirdM, theta3);
		System.out.format("1: %f, 2: %f, 3: %f\n", theta1, theta2, theta3);
	}
	
}