#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

const int DURATION_TO_CM = 58.2;
const int CM_TO_FEET = 30.48;

class UltrasonicSensor{
  private:
  // Pins
  int echoPin;
  int trigPin;
  
  // Send kill switch
  int tooCloseDistance;
  const int MINIMUM_RANGE = 1; // 5 cm
  int distance;
  long duration;

  public:
    // tooCloseDistance in feet
    UltrasonicSensor(int echoPin, int trigPin, int tooCloseDistance){
      this -> trigPin = trigPin;
      this -> echoPin = echoPin;
      // feet to cm
      this -> tooCloseDistance = tooCloseDistance * CM_TO_FEET;

      // set up pins
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    bool obstruction(){
      // Get the distance then check if an object is too close
      // Return tooClose (true, obstruction, false, no obstruction
      distance = getDistance();
      return (distance <= tooCloseDistance && distance >= MINIMUM_RANGE);
    }

    float getDistance(){
      /* The following trigPin/echoPin cycle is used to determine the
      distance of the nearest object by bouncing soundwaves off of it. */
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
  
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
  
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
  
      //Calculate the distance (in cm) based on the speed of sound.
      distance = duration/DURATION_TO_CM;
      return distance;

    }
};

#endif
