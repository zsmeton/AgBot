#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

static const int DURATION_TO_CM = 58.2;
static const int METERS_TO_CM = 100;

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
    // tooCloseDistance in meters
    UltrasonicSensor(int echoPin, int trigPin, int tooCloseDistance){
      this -> trigPin = trigPin;
      this -> echoPin = echoPin;
      // feet to cm
      this -> tooCloseDistance = tooCloseDistance * METERS_TO_CM;

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

    void setEchoPin(int pin){
      echoPin = pin;
    }

    void setTrigPin(int pin){
      echoPin = pin;
    }

    void setStopDist(int dist){
      tooCloseDistance = dist * METERS_TO_CM;
    }
};

#endif
