package ca.mcgill.ecse211.zipline;

import ca.mcgill.ecse211.zipline.UltrasonicLocalizer;
import ca.mcgill.ecse211.zipline.UltrasonicPoller;
import ca.mcgill.ecse211.zipline.Navigation;
import ca.mcgill.ecse211.zipline.LightLocalizer;
import ca.mcgill.ecse211.zipline.Odometer;
import ca.mcgill.ecse211.zipline.OdometryDisplay;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ZipLineLab {
	
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 10.3;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lsPort = LocalEV3.get().getPort("S2");
	public static final EV3LargeRegulatedMotor leftMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor pulleyMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static void main(String[] args) throws InterruptedException{
		
		
		int buttonChoice;
		final TextLCD t = LocalEV3.get().getTextLCD();
		final Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);
	
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
		final SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];
    
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(lsPort);
		SampleProvider colorValue = colorSensor.getMode("Red");
		float[] colorData = new float[3];
     
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, colorValue, colorData);
		int position=4;
		do{
			t.clear();
			t.drawString(" < Left   |   Right >", 0, 0);
			t.drawString("          |        ", 0, 1);
			t.drawString(" 0        |   1  ", 0, 2);
			t.drawString("          |        ", 0, 3);
			t.drawString("Up        |   Down    ", 0, 4);
			t.drawString("          |    ", 0, 5);
			t.drawString("2         |   3 ", 0, 6);
			buttonChoice = Button.waitForAnyPress();
		}  while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice !=
				Button.ID_UP && buttonChoice != Button.ID_DOWN);
		if(buttonChoice == Button.ID_LEFT){
			position = 0;
		}
		else if(buttonChoice == Button.ID_RIGHT){
			position = 1;
		}
		else if(buttonChoice == Button.ID_UP){
			position = 2;
		}
		else if(buttonChoice == Button.ID_DOWN){
			position =3;
		}
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(true, odometer, position);
		UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, usLocalizer);
		final TraverseZipLine trav = new TraverseZipLine(odometer, colorValue, colorData ); 
		odometer.start();
		odometryDisplay.start();
		usPoller.start();
  	  	buttonChoice = Button.waitForAnyPress();
	  	if(buttonChoice == Button.ID_ESCAPE){
	  		System.exit(0);
	  	}
		lightLocalizer.localize(position);
		final Navigation navigation = new Navigation(true);
	    leftMotor.forward();
	    leftMotor.flt();
	    rightMotor.forward();
	    rightMotor.flt();
	    Thread move = new Thread(){
  		  
  		  public void run() {
  			  navigation.travelTo(usDistance, odometer, WHEEL_RADIUS, WHEEL_RADIUS, TRACK,1
  				  ,6,2 ,6); 			  
  		  }
  		  
  	  	};
  	  	Thread traverse = new Thread(){
  	  		public void run(){
  	  			trav.traverse();
  	  		}
  	  	};
  	  	buttonChoice = Button.waitForAnyPress();
  	  	if(buttonChoice == Button.ID_ESCAPE){
  	  		System.exit(0);
  	  	}
  	  	move.start();  	
  	  	buttonChoice = Button.waitForAnyPress();
	  	if(buttonChoice == Button.ID_ESCAPE){
	  		System.exit(0);
	  	}
  	  	traverse.start();
  	  	traverse.join();
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	    System.exit(0);
	}

}	
