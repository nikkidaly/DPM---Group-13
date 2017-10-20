package ca.mcgill.ecse211.zipline;

import ca.mcgill.ecse211.zipline.ZipLineLab;
import lejos.hardware.Sound;
import ca.mcgill.ecse211.zipline.UltrasonicController;

public class UltrasonicLocalizer implements UltrasonicController {

	public static double distance;
	private static final int ROTATE_SPEED = 100;
	private boolean isFallingEdge;
	private Odometer odometer;
	private static int k = 1;
	private static int d = 30;
	private static boolean isAlphaOneSet;
	private static boolean isAlpha2Set;
	private static boolean isAlphaSet;
	private static boolean isFirstTurnDone;
	private static double alpha1;
	private static double alpha2;
	private static double alpha;
    
	private static boolean isBeta1Set;
	private static boolean isBeta2Set;
	private static boolean isBetaSet;
	private static boolean isdThetaSet;
	private static double beta1;
	private static double beta2;
	private static double beta;
	
	private static double minTheta;
    private static double minDistance;
	
	private static double dTheta;
	
	private static boolean keepGoing;
	private static boolean isInPosition;
	
	boolean isInitialized = false;
	private   int position;
	public UltrasonicLocalizer(boolean isFallingEdge, Odometer odometer, int position){
		this.isFallingEdge = isFallingEdge;
		this.odometer = odometer;
		this.position = position;
	}
	public static void fallingEdge(Odometer odometer, double distance){
		ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
	    ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);

	    if(distance<d+k && !isAlphaOneSet){
	    	alpha1 = odometer.getTheta();
	    	isAlphaOneSet = true;
	    }
	    
	    if(distance<d-k && isAlphaOneSet && !isAlpha2Set){
	    	alpha2 = odometer.getTheta();
	    	isAlpha2Set = true;
	    }
	      
	    if(isAlphaOneSet && isAlpha2Set){
	    	alpha = (alpha1 + alpha2)/2;
	    	isAlphaSet = true;
	    }
	    
	    if(isAlphaSet && distance>250){
	    	isFirstTurnDone = true;
	    	
	    } else {
	    	ZipLineLab.leftMotor.rotate(convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, 360.0), true);
	    	ZipLineLab.rightMotor.rotate(-convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, 360.0), true);
	    }
	      
	    if(distance<d+k && isFirstTurnDone && !isBeta1Set){
	    	beta1 = odometer.getTheta();
	    	isBeta1Set = true;
	    }
	      
	    if(distance<d-k && isBeta1Set && !isBeta2Set){
	    	beta2 = odometer.getTheta();
	    	isBeta2Set = true;
	    }
	   	  
	    if(isBeta1Set && isBeta2Set){
	   		beta = (beta1+beta2)/2;
	       isBetaSet = true;
	       
	   	}
	   
	    if(isFirstTurnDone && !isBetaSet){
	    	ZipLineLab.leftMotor.rotate(convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -360.0), true);
	    	ZipLineLab.rightMotor.rotate(-convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -360.0), true);
	    }
	    
	    if(isAlphaSet && isBetaSet){
	    	if(alpha<beta){
	    		dTheta = 0- (alpha+beta)/2;
	    		isdThetaSet = true;
	    	} else {
	    		dTheta = 170- (alpha+beta)/2;
	   			isdThetaSet = true;
	   		  }
	   	 }	   	  	   	  
		}
	//Sets the robot to the correct 0 degree angle
	public  void adjust(Odometer odometer){
	  
  	  if(isdThetaSet){
  		  while (dTheta>360){
  			  dTheta = dTheta - 360;
  		  }
  		  
  		  ZipLineLab.rightMotor.rotate(convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -odometer.getTheta()-dTheta), true);
  		  ZipLineLab.leftMotor.rotate(-convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -odometer.getTheta()-dTheta), false);
  		  if(this.position ==1 || this.position ==3){
  			  ZipLineLab.rightMotor.rotate(convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -90), true);
  	  		  ZipLineLab.leftMotor.rotate(-convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -90), false);
  		  }
  		  odometer.setTheta(0);
  		  keepGoing = false;
  	  }
	}
	// initialize all the global variables
	private void initialize(){
		isAlphaOneSet = false;
		isAlpha2Set = false;
		isAlphaSet = false;
		isFirstTurnDone = false;
		alpha1 = 0;
		alpha2 = 0;
		alpha = 0;
		
		minTheta =  10;
		minDistance = 250;
		keepGoing = true;
	    
		isBeta1Set = false;
		isBeta2Set = false;
		isBetaSet = false;
		isdThetaSet = false;
		beta1 = 0;
		beta2 = 0;
		beta = 0;
	    
		
		dTheta = 0;
		isInPosition = false;
	}
	
	public static void risingEdge(Odometer odometer, double distance){
		  ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
	      ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
	      
	      if(distance > d-k && !isAlphaOneSet){
	    	  alpha1 = odometer.getTheta();
	    	  isAlphaOneSet = true;
	      }
	      
	      if(distance>d+k && isAlphaOneSet && !isAlpha2Set){
	    	  alpha2 = odometer.getTheta();
	    	  isAlpha2Set = true;
	      }
	      
	      if(isAlphaOneSet && isAlpha2Set){
	    	  alpha = (alpha1 + alpha2)/2;
	    	  isAlphaSet = true;
	      }
	      //Turn until the first rising edge is detected
	      if(isAlphaSet && distance>minDistance){
	    	  isFirstTurnDone = true;
	      } else {
		      ZipLineLab.leftMotor.rotate(convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, 360.0), true);
	    	  ZipLineLab.rightMotor.rotate(-convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, 360.0), true);
	      }
	      
	      if(isFirstTurnDone && odometer.getTheta()<minTheta){
	    	  isInPosition =true;
	      }
	      
	      if(distance>d-k && isFirstTurnDone && !isBeta1Set && isInPosition){
	    	  beta1 = odometer.getTheta();
	    	  isBeta1Set = true;
	      }
	      
	      if(distance>d+k && isBeta1Set && !isBeta2Set){
	    	  beta2 = odometer.getTheta();
	    	  isBeta2Set = true;
	      }
	      
	   	  if(isBeta1Set && isBeta2Set){
	   		  beta = (beta1+beta2)/2;
	   		  isBetaSet = true;
	   	  }
	   	  
	   	  //turn until the second rising edge is detected
	   	  if(isFirstTurnDone && !isBetaSet){
	    	  ZipLineLab.leftMotor.rotate(convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -360.0), true);
	    	  ZipLineLab.rightMotor.rotate(-convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, -360.0), true);
	      }

	   	  if(isAlphaSet && isBetaSet){
	   		  if(alpha<beta){
	   			  dTheta = 115- (alpha+beta)/2;
	   			  isdThetaSet = true;
	   		  }
	   		  else{
	   			  dTheta = 295- (alpha+beta)/2;
	   			  isdThetaSet = true;
	   		  }
	   	  }	  
	}
	
	//methods borrowed from pervious lab
	private static int convertDistance(double radius, double distance) {
	   return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
    private static int convertAngle(double radius, double width, double angle) {
    	return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    
	@Override	
	public void processUSData(int distance) {
		
		if(!isInitialized){
//			if(distance>200){
//				this.isFallingEdge = true;
//			}
//			else{
//				this.isFallingEdge = false;
//			}
			initialize();
			isInitialized = true;
		}
		
		this.distance = distance;
		if(isFallingEdge && keepGoing){
			if(isBetaSet && isAlphaSet){
				
				adjust(odometer);
				
			}else{
				fallingEdge(this.odometer, distance);	
			}
			
		}
		if(!isFallingEdge && keepGoing){
			if(isBetaSet && isAlphaSet){
				adjust(odometer);
			}
			else{
				risingEdge(this.odometer, distance);
			}
		}
		
	}
	@Override
	public int readUSDistance() {
		// 
		return 0;
	}
	
}
