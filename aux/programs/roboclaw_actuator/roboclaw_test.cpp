/**
 * Basicmicro Library Example: M1PPOS (125)
 *
 * Demonstrates commanding Motor 1 to move to a position specified as a
 * percentage of the configured position range.
 *
 * This command requires Position PID control to be enabled and tuned for Motor 1,
 * and encoders must be configured and connected. The percentage position is a
 * signed 16-bit value (-32768 to +32767).
 *
 * This example uses HardwareSerial (Serial1) for communication with the controller
 * and HardwareSerial (Serial) for debugging output. Adjust serial ports and
 * controller address as needed for your setup.
 */

// #include <Arduino.h>
// #include <Basicmicro.h>
#include "roboclaw.hpp"
#include <cstdint>
#include <cstdio>
// #include <chrono>

// uint64_t millis() {
//     using namespace std::chrono;
//     return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
// }

RoboClaw controller = RoboClaw("/dev/ttyACM0", 34800, 10000);

// Define example target positions for Motor 1 as percentage (signed 16-bit)
// -32768 is minimum configured position, +32767 is maximum configured position.
int16_t target_percent_pos_1 = 16383;  // Example: ~50% of range (corresponds to (min+max)/2 + (max-min)/2 * 0.5)
int16_t target_percent_pos_2 = -16383; // Example: ~-50% of range (corresponds to (min+max)/2 - (max-min)/2 * 0.5)
int16_t target_percent_pos_3 = 0;      // Example: 0% (corresponds to the center of the range (min+max)/2)
const uint8_t MOTOR_ADDRESS = 0x80;

// Define buffer control flag (0 = Immediate Execution, 1 = Add to Buffer)
uint8_t buffer_mode = 0; // Example: Execute command immediately
uint8_t rc = 0;

int main() {
   // Initialize debug serial port

   //Serial.println("Basicmicro M1PPOS Example");
   //Serial.print("Connecting to controller on ");
   printf("Turning on roboclaw\n");
   usleep(1000*1000);
   
   char versionBuf[128];
   rc = controller.ReadVersion(MOTOR_ADDRESS, versionBuf);
   if (rc == 1) {
   printf("RoboClaw Version: %s\n", versionBuf);
   } else {
      printf("Could not get RoboClaw Version\n");
   }

   // Note: For Percentage Position commands to work, Position PID must be enabled and tuned.
   // Encoders must be configured (e.g., Quadrature mode), AND the position MIN/MAX range
   // must be set using SETM1POSPID (or SETM2POSPID for M2). The controller maps the
   // -32768 to +32767 percentage range to your configured encoder MIN/MAX range.
   // You might need to send SETM1POSPID and SETM1ENCODERMODE here or ensure they are configured in NVM.
   // Example Position PID values and range (placeholders):
   // controller.SetM1PositionPID(MOTOR_ADDRESS, 0.5, 0.05, 0.005, 10000, 10, 0, 100000); // Sets min=0, max=100000 for percentage mapping

   printf("Attempting to command Motor 1 to initial percentage position: \n");
   printf("%d (out of -32768 to 32767)\n",target_percent_pos_1);

   //  Serial.print(" (Buffer mode: "); Serial.print(buffer_mode); Serial.println(")");
   printf(" (Buffer mode: %d)\n", buffer_mode);

   int success;

   while (1) {
      static unsigned long lastCommandTime = 0;
      static int command_state = 0; // 0: pos1, 1: pos2, 2: pos3

      // Check controller status or use timing to know when the previous command is done
      // before sending a new one, especially when using immediate execution (buffer_mode=0).
      // For simplicity in this example, we'll just use a delay to space out commands.
      if (tnow()/1000 - lastCommandTime > 5000) { // Wait 5 seconds before sending the next command
         lastCommandTime = tnow()/1000;

         int16_t current_target_percent_pos;

         switch(command_state) {
            case 0:
               current_target_percent_pos = target_percent_pos_2;
               command_state = 1;
               break;
            case 1:
               current_target_percent_pos = target_percent_pos_3;
               command_state = 2;
               break;
            case 2:
               current_target_percent_pos = target_percent_pos_1;
               command_state = 0;
               break;
         }

         //printf("Commanding Motor 1 to percentage position: ")
         printf("Commanding Motor 1 to percentage position: %d (out of -32768 to 32767)\n",current_target_percent_pos);
         //Serial.print(current_target_percent_pos); Serial.println(" (out of -32768 to 32767)");

         // Variables to store the read PID parameters
         float read_Kp, read_Ki, read_Kd;
         uint32_t read_KiMax, read_DeadZone, read_Min, read_Max;

         // Attempt to read the Position PID parameters for Motor 1
         // The function returns true on success, false on failure.
         // The parameters are stored in the provided variables.
         success = controller.ReadM1PositionPID(MOTOR_ADDRESS, read_Kp, read_Ki, read_Kd, read_KiMax, read_DeadZone, read_Min, read_Max);

         if (success) {
            printf("READM1POSPID command successful.\n");
            printf("Motor 1 Position PID Parameters:\n");
            printf("  Kp: %.4f\n",read_Kp); // Print with 4 decimal places
            printf("  Ki: %.4f\n",read_Ki);
            printf("  Kd: %.4f\n",read_Kd);
            printf("  KiMax: %u", read_KiMax);
            printf("  DeadZone: %u", read_DeadZone);
            printf("  MinPos: %u", read_Min);
            printf("  MaxPos: %u", read_Max);
         } else {
            printf("READM1POSPID command failed.");
            printf("Check wiring, power, address, and baud rate.");
         }

         // Attempt to send the M1PercentPosition command
         // The position parameter expects uint16_t, so we cast our signed int16_t.
         success = controller.M1PercentPosition(MOTOR_ADDRESS, current_target_percent_pos, buffer_mode);

         if (success) {
            printf("M1PPOS command successful. Motor should start moving.\n");
            // In a real application, you might monitor position error or speed to know when it's stopped.
         } else {
            printf("M1PPOS command failed.\n");
            printf("Check wiring, power, address, baud rate, and controller configuration (Is Position PID enabled and range set?).\n");
         }
      }

      // Short delay in the loop
      //delay(50);
      usleep(50*1000);
   }
}
