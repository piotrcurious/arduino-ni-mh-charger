#include <TaskFun.h>
#include <Kalman.h>

// Define the pins for the temperature sensors
#define TEMP1_PIN A0
#define TEMP2_PIN A1
#define TEMP3_PIN A2

// Define the time interval for reading and filtering the temperatures (in milliseconds)
#define INTERVAL 100

// Define the time constant for the ambient temperature effect (in milliseconds)
#define TAU 1000

// Create Kalman filter objects for each temperature sensor
Kalman filter1;
Kalman filter2;
Kalman filter3;

// Create Kalman filter objects for the deltas of each sensor temp and the ambient temp
Kalman delta1;
Kalman delta2;
Kalman delta3;

// Declare global variables for the temperature readings and the filtered values
// Use syncVar to synchronize access to shared variables
syncVar<float> temp1, temp2, temp3;
syncVar<float> filtered1, filtered2, filtered3;

// Declare global variables for the deltas and the filtered deltas
// Use syncVar to synchronize access to shared variables
syncVar<float> delta1, delta2, delta3;
syncVar<float> filtered_delta1, filtered_delta2, filtered_delta3;

// Declare a global variable for the ambient temperature effect
// Use syncVar to synchronize access to shared variables
syncVar<float> ambient_effect;

// Declare a function prototype for the task that reads the temperatures
void read_temps();

// Declare a function prototype for the task that filters the temperatures
void filter_temps();

// Initialize the tasks
TaskFun read_task(read_temps, INTERVAL);
TaskFun filter_task(filter_temps, INTERVAL);

void setup() {
  // Initialize the serial monitor
  Serial.begin(9600);

  // Initialize the temperature sensors
  pinMode(TEMP1_PIN, INPUT);
  pinMode(TEMP2_PIN, INPUT);
  pinMode(TEMP3_PIN, INPUT);

  // Initialize the ambient temperature effect
  ambient_effect = 0;

  
  // Initialize the tasks by calling setupTasks()
  setupTasks();
  // Run the tasks
  read_task.run();
  filter_task.run();
  
}

void loop() {
}

// Define the function that reads the temperatures
void read_temps() {
  // Read the analog values from the sensors
  int raw1 = analogRead(TEMP1_PIN);
  int raw2 = analogRead(TEMP2_PIN);
  int raw3 = analogRead(TEMP3_PIN);

  // Convert the raw values to Celsius degrees
  // Assuming a linear relationship between voltage and temperature
  // and a 10-bit ADC with a reference voltage of 5V
  temp1 = raw1 * 0.00488 * 100;
  temp2 = raw2 * 0.00488 * 100;
  temp3 = raw3 * 0.00488 * 100;

  // Calculate the deltas of each sensor temp and the ambient temp
  delta1 = temp1 - temp3;
  delta2 = temp2 - temp3;
  delta3 = temp3 - temp3;
}

// Define the function that filters the temperatures
void filter_temps() {
  // Calculate the ambient temperature effect
  // Assuming an exponential decay function
  ambient_effect = ambient_effect * exp(-INTERVAL / TAU) + temp3 * (1 - exp(-INTERVAL / TAU));

  // Filter the temperatures using the Kalman filter
  // Assuming the process noise and the measurement noise are both 0.1
  // and the initial estimate is the first reading
  filtered1 = filter1.updateEstimate(temp1, 0.1, 0.1);
  filtered2 = filter2.updateEstimate(temp2, 0.1, 0.1);
  filtered3 = filter3.updateEstimate(temp3, 0.1, 0.1);

  // Filter the deltas using the Kalman filter
  // Assuming the process noise and the measurement noise are both 0.1
  // and the initial estimate is the first delta
  // Use the ambient_effect to reduce the Kalman gain only for the deltas
  filtered_delta1 = delta1.updateEstimate(delta1, 0.1 * (1 - ambient_effect), 0.1);
  filtered_delta2 = delta2.updateEstimate(delta2, 0.1 * (1 - ambient_effect), 0.1);
  filtered_delta3 = delta3.updateEstimate(delta3, 0.1, 0.1);

  // Print the filtered values and the filtered deltas to the serial monitor
  Serial.print("Filtered1: ");
  Serial.print(filtered1);
  Serial.print(" Filtered2: ");
  Serial.print(filtered2);
  Serial.print(" Filtered3: ");
  Serial.print(filtered3);
  Serial.print(" Filtered_delta1: ");
  Serial.print(filtered_delta1);
  Serial.print(" Filtered_delta2: ");
  Serial.print(filtered_delta2);
  Serial.print(" Filtered_delta3: ");
  Serial.println(filtered_delta3);
}
