/*

PID Controller

*/

class PID {
  private:
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    int inputPin = 0;
    int outputPin = 0;

    float error = 0;
    float previousError = 0;
    float integral = 0;
    int targetValue = 200;
    float lastTime = 0;
    

  public:
    PID(float Kp = 0, float Ki = 0, float Kd = 0, int inputPin = 0, int outputPin = 0) {
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
      this->inputPin = inputPin;
      this->outputPin = outputPin;
    }
    /*Tuning the PID values*/
    void setPID(float Kp, float Ki, float Kd) {
      this->Kp = Kp;
      this->Ki = Ki;
      this->Kd = Kd;
    }
    /*Set the Input and Output Pins*/
    void setPins(int inputPin, int outputPin) {
      this->inputPin = inputPin;
      this->outputPin = outputPin;
    }
    /*Set the Target Value*/
    void setTargetValue(int target) {
      this->targetValue = target;
    }

    /* Get the (Delta t) for Drivative function */
    float getDt() {
      double now = millis();
      float dt = (now - this->lastTime) / 1000.0;
      this->lastTime = now;
      return dt;
    }
    /*Get the Error Value*/
    float getError() {
      float actualValue = analogRead(inputPin); 
      error = targetValue - actualValue;
      return error;
    }

    /*The Main Controller*/
    float PID_controller() {
      // Get the error
      float error = getError();
      // get the dt
      float dt = getDt();


      // Proportional term
      float proportional = error;

      // Integral term
      integral += error * dt;

      // Derivative term
      float derivative = (error - previousError) / dt;
      previousError = error;

      // Calculate the PID output
      float output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

      // Output the result
      analogWrite(outputPin, (int)output);

      return output;
    }
};


// EXAMPLE :

// Define the input & output pins
#define INPUTPIN A0
#define OUTPUTPIN 9

// Create the Controller
PID myPID(1.2, 2.0, 2.5, INPUTPIN, OUTPUTPIN);

void setup() {
  // Set the target 
  myPID.setTargetValue(200);
}

void loop() {
  // run The controller
  myPID.PID_controller();
  delay(25);
}
