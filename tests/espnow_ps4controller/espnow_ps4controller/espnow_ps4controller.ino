#include <Bluepad32.h>
#include <WiFi.h>
#include <esp_now.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
uint8_t flightMAC[] = {0x1C, 0xDB, 0xD4, 0x86, 0x35, 0x64};

typedef struct {
    int16_t throttle;
    int16_t brake;
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint16_t buttons;
    uint8_t dpad;
    bool transmitter_connection;
} controller_packet_t;

typedef struct {  
    float roll;
    float pitch;
    float yaw;
    float roll_sp;
    float pitch_sp;
    float yaw_rate_sp;
    float roll_kp, roll_kd, roll_ki;
    float pitch_kp, pitch_kd, pitch_ki;
    float yaw_kp, yaw_kd, yaw_ki;
    float roll_trim;
    float pitch_trim;
    uint8_t pid_axis;
    uint8_t pid_param;
    uint8_t pid_loop;   // 0 = angle, 1 = rate
    bool receiver_connection;
} telemetry_packet_t;

controller_packet_t txPacket;
telemetry_packet_t rxTelemetry;

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








void dumpGamepad(ControllerPtr ctl) {
    txPacket.throttle = ctl->throttle();
    txPacket.brake = ctl->brake();
    txPacket.rx = ctl->axisRX();
    txPacket.ry = ctl->axisRY();
    txPacket.lx = ctl->axisX();
    txPacket.ly = ctl->axisY();
    txPacket.buttons = ctl->buttons();
    txPacket.dpad = ctl->dpad();
    txPacket.transmitter_connection = false;

    esp_now_send(flightMAC, (uint8_t *)&txPacket, sizeof(txPacket));

    // Serial.printf("T:%d B:%d LX:%d LY:%d RX:%d RY:%d BTN:0x%04X DPAD:0x%02X\n",
    //     txPacket.throttle, txPacket.brake, txPacket.lx, txPacket.ly,
    //     txPacket.rx, txPacket.ry, txPacket.buttons, txPacket.dpad);
}






void processGamepad(ControllerPtr ctl) {
    dumpGamepad(ctl);
}






//this section is likely not needed //

// ========= GAME CONTROLLER ACTIONS SECTION ========= //



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
  Serial.begin(115200); 

WiFi.mode(WIFI_STA);
esp_now_init();

esp_now_peer_info_t peer = {};
memcpy(peer.peer_addr, flightMAC, 6);
peer.channel = 0;
peer.encrypt = false;
esp_now_add_peer(&peer);

esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(telemetry_packet_t)) {
        memcpy(&rxTelemetry, data, sizeof(telemetry_packet_t));
    }
});



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


  static uint32_t last_print = 0;
  if (millis() - last_print >= 200) {
      last_print = millis();
      const char* axes[] = {"Roll", "Pitch", "Yaw"};
      const char* params[] = {"Kp", "Kd", "Ki"};


      // const char* loop = (rxTelemetry.pid_loop == 1) ? "RATE" : "ANGLE";

      // const char* loop;
      // if (rxTelemetry.pid_axis == 2) loop = "YAW";
      // else loop = (rxTelemetry.pid_loop == 1) ? "RATE" : "ANGLE";

      const char* loop;
if (rxTelemetry.pid_loop == 2) loop = "TRIM";
else if (rxTelemetry.pid_axis == 2) loop = "YAW";
else loop = (rxTelemetry.pid_loop == 1) ? "RATE" : "ANGLE";
      
      // Serial.printf("T:%d B:%d LX:%d LY:%d RX:%d RY:%d BTN:0x%04X DPAD:0x%02X\n", txPacket.throttle, txPacket.brake, txPacket.lx, txPacket.ly, txPacket.rx, txPacket.ry, txPacket.buttons, txPacket.dpad);
      // Serial.printf("R:%.1f P:%.1f Y:%.1f | Editing: %s %s\n", rxTelemetry.roll, rxTelemetry.pitch, rxTelemetry.yaw, axes[rxTelemetry.pid_axis], params[rxTelemetry.pid_param]);
      // Serial.printf("Roll  Kp:%.3f Kd:%.3f Ki:%.3f\n", rxTelemetry.roll_kp, rxTelemetry.roll_kd, rxTelemetry.roll_ki);
      // Serial.printf("Pitch Kp:%.3f Kd:%.3f Ki:%.3f\n", rxTelemetry.pitch_kp, rxTelemetry.pitch_kd, rxTelemetry.pitch_ki);
      // Serial.printf("Yaw   Kp:%.3f Kd:%.3f Ki:%.3f\n", rxTelemetry.yaw_kp, rxTelemetry.yaw_kd, rxTelemetry.yaw_ki);
      // printf("[%s] %s %s = %.3f\n", loop, axis_str, param_str, value);

      float value = 0.0f;

if (rxTelemetry.pid_axis == 0) {
    if (rxTelemetry.pid_param == 0) value = rxTelemetry.roll_kp;
    else if (rxTelemetry.pid_param == 1) value = rxTelemetry.roll_kd;
    else value = rxTelemetry.roll_ki;
}
else if (rxTelemetry.pid_axis == 1) {
    if (rxTelemetry.pid_param == 0) value = rxTelemetry.pitch_kp;
    else if (rxTelemetry.pid_param == 1) value = rxTelemetry.pitch_kd;
    else value = rxTelemetry.pitch_ki;
}
else {
    if (rxTelemetry.pid_param == 0) value = rxTelemetry.yaw_kp;
    else if (rxTelemetry.pid_param == 1) value = rxTelemetry.yaw_kd;
    else value = rxTelemetry.yaw_ki;
}
    // if(rxPacket.transmitter_connection){
    //     txTelemetry.receiver_connection = true;
    // } else {
    //     txTelemetry.receiver_connection = false;
    // }

    // if(rxTelemetry.receiver_connection){
    //   txPacket.transmitter_connection = true;
    // }
    // else{
    //   txPacket.transmitter_connection = false;  
    // }
    


    // txPacket.throttle = ctl->throttle();
    // txPacket.brake = ctl->brake();
    // txPacket.rx = ctl->axisRX();
    // txPacket.ry = ctl->axisRY();
    // txPacket.lx = ctl->axisX();
    // txPacket.ly = ctl->axisY();
    // txPacket.buttons = ctl->buttons();
    // txPacket.dpad = ctl->dpad();
Serial.printf("\n\n Data Sent to Flight Controller:\n\n");
Serial.printf("T:%d   B: %d   Rx: %d   Ry: %d   Lx: %d   Ly: %d   Btn: %x   Dpad: %x", 
      txPacket.throttle, txPacket.brake, txPacket.rx, txPacket.ry, txPacket.lx, txPacket.ly, txPacket.buttons, txPacket.dpad);
    

Serial.printf("\n\n Data Received from Flight Controller: \n\n");

Serial.printf("Roll: pv:%.1f | sp: %.1f  Pitch: pv:%.1f | sp: %.1f Yaw Rate: pv:%.1f | sp: %.1f\n", rxTelemetry.pitch, rxTelemetry.roll_sp,rxTelemetry.roll, rxTelemetry.pitch_sp, rxTelemetry.yaw, rxTelemetry.yaw_rate_sp);
// Serial.printf("Pitch:%.1f Roll:%.1f Yaw Rate:%.1f\n",rxTelemetry.roll, rxTelemetry.pitch, rxTelemetry.yaw);
Serial.printf("[%s] Editing: %s %s = %.3f\n",loop,axes[rxTelemetry.pid_axis],params[rxTelemetry.pid_param], value);

Serial.printf("Roll  Kp:%.3f Kd:%.3f Ki:%.3f\n",rxTelemetry.roll_kp, rxTelemetry.roll_kd, rxTelemetry.roll_ki);
Serial.printf("Pitch Kp:%.3f Kd:%.3f Ki:%.3f\n",rxTelemetry.pitch_kp, rxTelemetry.pitch_kd, rxTelemetry.pitch_ki);
Serial.printf("Yaw   Kp:%.3f Kd:%.3f Ki:%.3f\n",rxTelemetry.yaw_kp, rxTelemetry.yaw_kd, rxTelemetry.yaw_ki);
Serial.printf("Trim R:%.1f P:%.1f\n", rxTelemetry.roll_trim, rxTelemetry.pitch_trim);
  }

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
  //delay(1);
}