/*

PID Controller
-- take the Feedback Value 
-- Compute the output
-- Return the output

*/
#include <arduino.h>

class PID {
  private:
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    
    float error = 0;
    float previousError = 0;
    float integral = 0;
    int targetValue = 0;
    float lastTime = 0;
    

  public:
    PID(float Kp = 0, float Ki = 0, float Kd = 0, float targetValue = 0) {
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
      this->targetValue = targetValue;
    }
    /* To change the Tuning values of the PID */
    void setPID(float Kp, float Ki, float Kd) {
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
    }

    /*Set the Target Value*/
    void setTargetValue(int target) {
      this->targetValue = target;
    }

    /*The Main Controller*/
    float compute(float feedback) {
      // Get the error from feedback
      error = this->targetValue - feedback;

      // get the dt
      double now = millis();
      float dt = (now - this->lastTime) / 1000.0;

      // Proportional term
      float proportional = error;

      // Integral term
      integral += error * dt;

      // Derivative term
      float derivative = (error - previousError) / dt;

      previousError = error;

      // Calculate the PID output
      float output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

      // Update The lastTime 
      this->lastTime = now;

      return output;
    }
};


// EXAMPLE :

// Define the input & output pins
#define INPUTPIN A0
#define OUTPUTPIN 9

// Create the Controller
PID myPID(1.2, 2.0, 2.5);

void setup() {
  // // Set the target 
  myPID.setTargetValue(200);
}

void loop() {
  // Get the feedback
  feedback = "feedback from a sensor";

  output = myPID.compute(feedback);
  anaglogWrite(OUTPUTPIN, output)
  delay(25);
}
