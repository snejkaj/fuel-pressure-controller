/*working variables*/
unsigned long lastTime;
double input, output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
void Compute()
{
/*How long since we last calculated*/
unsigned long now = millis();
double timeChange = (double)(now - lastTime);
 
/*Compute all the working error variables*/
double error = Setpoint - input;
errSum += (error * timeChange);
double dErr = (error - lastErr) / timeChange;
 
/*Compute PID Output*/
output = kp * error + ki * errSum + kd * dErr;
 
/*Remember some variables for next time*/
lastErr = error;
lastTime = now;
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
kp = Kp;
ki = Ki;
kd = Kd;
}
