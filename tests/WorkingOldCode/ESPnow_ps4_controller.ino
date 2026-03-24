#include <Bluepad32.h>
#include <HardwareSerial.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define RX_PIN 19
#define TX_PIN 17

#define BT_BAUD 9600

HardwareSerial PicoSerial(1);


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}




// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

//We will use this section to send controller data over UART//


void dumpGamepad(ControllerPtr ctl) {

//sends start bit and splits uint32_t into 4 bytes


      uint32_t throttle_value = ctl->throttle();
      uint32_t brake_value = ctl->brake();
      int32_t rx = ctl->axisRX();
      int32_t ry = ctl->axisRY();
      int32_t lx = ctl->axisX();
      int32_t ly = ctl->axisY();

      // Serial.print("\nStart byte: ");
      // Serial.print(0xFF);
      // Serial.print("   ");

      PicoSerial.write(0xFF);
      PicoSerial.write((throttle_value >> 24) & 0xFF);  // First byte
      PicoSerial.write((throttle_value >> 16) & 0xFF);  // Second byte
      PicoSerial.write((throttle_value >> 8) & 0xFF);   // Third byte
      PicoSerial.write(throttle_value & 0xFF);   // Fourth byte
      
      PicoSerial.write((brake_value>>24)&0xFF);
      PicoSerial.write((brake_value>>16)&0xFF);
      PicoSerial.write((brake_value>>8)&0xFF);
      PicoSerial.write((brake_value)&0xFF);       

      PicoSerial.write((rx>>24)&0xFF);
      PicoSerial.write((rx>>16)&0xFF);
      PicoSerial.write((rx>>8)&0xFF);
      PicoSerial.write((rx)&0xFF);

      PicoSerial.write((ry>>24)&0xFF);
      PicoSerial.write((ry>>16)&0xFF);
      PicoSerial.write((ry>>8)&0xFF);
      PicoSerial.write((ry)&0xFF);

      PicoSerial.write((lx>>24)&0xFF);
      PicoSerial.write((lx>>16)&0xFF);
      PicoSerial.write((lx>>8)&0xFF);
      PicoSerial.write((lx)&0xFF);

      PicoSerial.write((ly>>24)&0xFF);
      PicoSerial.write((ly>>16)&0xFF);
      PicoSerial.write((ly>>8)&0xFF);
      PicoSerial.write((ly)&0xFF);


  Serial.printf("\n--------------------------------------------------------\n");
  Serial.printf("Throttle: %d  Brake: %d Lx: %d Ly: %d Rx: %d Ry: %d ",ctl->throttle(),ctl->brake(),ctl->axisX(),ctl->axisY(),ctl->axisRX(),ctl->axisRY());
  Serial.printf("\n--------------------------------------------------------\n");


/*
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ() // Accelerometer Z
  ); */        
}

/*
mySerial.printf(
  //"idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  //"misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}
*/






















//this section is likely not needed //

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
 
  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(9600); 

  PicoSerial.begin(9600,SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.printf("\n");
  Serial.printf("Code for PS4 Controller Link");
  Serial.printf("\n");

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);


 // mySerial.printf("Serial 2 started at %d baud rate",BT_BAUD);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
   
  // Send message over UART
  //mySerial.println(counter);
  
  //Serial.println( "Sent: " + String(counter));
  // increment the counter
    
    // vTaskDelay(1);
  delay(1);
}