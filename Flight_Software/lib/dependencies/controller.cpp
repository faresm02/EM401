#include <controller.h>
#include <WiFi.h>
#include <esp_now.h>
#include <pids.h>


uint8_t controllerMAC[] = {0xCC, 0x7B, 0x5C, 0x25, 0xF2, 0x24};
controller_packet_t rxPacket;
telemetry_packet_t txTelemetry;
volatile bool newData = false;
uint8_t pid_axis = 0;
uint8_t pid_param = 0;
bool r1_prev = false;
bool l1_prev = false;
bool use_rate_loop = true;   // start with rate loop
bool square_prev = false;
uint8_t dpad_prev = 0;
float roll_trim = 0.0f;
float pitch_trim = 0.0f;
bool trim_mode = false;
bool x_prev = false;

pid_vars *active_pid = nullptr;

const char* active_axis_str = "ROLL";
const char* active_loop_str = "RATE";
const char* active_param_str = "Kp";

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(controller_packet_t)) {
        memcpy(&rxPacket, data, sizeof(controller_packet_t));
        newData = true;
    }
}

void controller_espnow_init() {
    printf("Hello ESP-NOW!\n");
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(onReceive);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, controllerMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    printf("Receiver ready\n");
}

// // void process_controller_data(){
// // void process_controller_data(pid_vars *roll_pid, pid_vars *pitch_pid, pid_vars *yaw_pid){
//     if (newData) {
//         newData = false;

//     // R1 - cycle parameter
//     bool r1 = rxPacket.buttons & 0x0020;
//     if (r1 && !r1_prev) {
//         pid_param = (pid_param + 1) % 3;
//     }
//     r1_prev = r1;

//     // L1 - cycle axis
//     bool l1 = rxPacket.buttons & 0x0010;
//     if (l1 && !l1_prev) {
//         pid_axis = (pid_axis + 1) % 3;
//     }
//     l1_prev = l1;

//     // D-pad adjustment
//     uint8_t dpad = rxPacket.dpad;
//     if (dpad != dpad_prev) {
//         float adj = 0;
//         if (dpad & 0x01) adj = 0.01f;   // up
//         if (dpad & 0x02) adj = -0.01f;  // down
//         if (dpad & 0x04) adj = 0.1f;    // right
//         if (dpad & 0x08) adj = -0.1f;   // left

//         if (adj != 0) {
//             // printf("PID adj: axis=%d param=%d delta=%.2f\n", pid_axis, pid_param, adj);
//             // TODO: apply adj to actual PID vars when integrated into flight code
//         }
//     }
//     dpad_prev = dpad;

//     // printf("T:%d B:%d LX:%d LY:%d RX:%d RY:%d BTN:0x%04X DPAD:0x%02X\n",
//     //     rxPacket.throttle, rxPacket.brake, rxPacket.lx, rxPacket.ly,
//     //     rxPacket.rx, rxPacket.ry, rxPacket.buttons, rxPacket.dpad);

//     // Send telemetry back
//     txTelemetry.roll = 0;      // placeholder until integrated
//     txTelemetry.pitch = 0;
//     txTelemetry.yaw = 0;
//     txTelemetry.pid_axis = pid_axis;
//     txTelemetry.pid_param = pid_param;
//     txTelemetry.pid_value = 0; // placeholder
//     esp_now_send(controllerMAC, (uint8_t *)&txTelemetry, sizeof(txTelemetry));
//     }

// }


//void process_controller_data(pid_vars *roll_pid, pid_vars *pitch_pid, pid_vars *yaw_pid, float roll, float pitch, float yaw){
void process_controller_data(pid_vars *roll_angle,pid_vars *roll_rate,pid_vars *pitch_angle,pid_vars *pitch_rate,pid_vars *yaw_rate,float roll, float pitch, float yaw){  

if (newData) {
        newData = false;

        // R1 - cycle parameter
        bool r1 = rxPacket.buttons & 0x0020;
        if (r1 && !r1_prev) {
            pid_param = (pid_param + 1) % 3;
        }
        r1_prev = r1;

        // L1 - cycle axis
        bool l1 = rxPacket.buttons & 0x0010;
        if (l1 && !l1_prev) {
            pid_axis = (pid_axis + 1) % 3;
        }
        l1_prev = l1;

        bool square = rxPacket.buttons & 0x0004; // <-- replace with your actual Square mask
        if (square && !square_prev) {
            use_rate_loop = !use_rate_loop;
        }
        square_prev = square;

        bool x_btn = rxPacket.buttons & 0x0001;
        if (x_btn && !x_prev) {
            trim_mode = !trim_mode;
        }
        x_prev = x_btn;

        // Pick the active PID struct
        // pid_vars *active;
        // if (pid_axis == 0) active = roll_pid;
        // else if (pid_axis == 1) active = pitch_pid;
        // else active = yaw_pid;

pid_vars *active;

        if (pid_axis == 0) {  // Roll
            active = use_rate_loop ? roll_rate : roll_angle;
        }
        else if (pid_axis == 1) {  // Pitch
            active = use_rate_loop ? pitch_rate : pitch_angle;
        }
        else {  // Yaw
            active = yaw_rate;
        }        

        active_pid = active;

        // Axis
        if (pid_axis == 0) active_axis_str = "ROLL";
        else if (pid_axis == 1) active_axis_str = "PITCH";
        else active_axis_str = "YAW";



        if (trim_mode){
            active_loop_str = "TRIM";
        // Loop
        }else if (pid_axis == 2)
        {
            active_loop_str = "YAW";
        }
        else
        {
            active_loop_str = use_rate_loop ? "RATE" : "ANGLE";
        }

        // Param
        if (pid_param == 0) active_param_str = "Kp";
        else if (pid_param == 1) active_param_str = "Kd";
        else active_param_str = "Ki";

        // D-pad adjustment
        uint8_t dpad = rxPacket.dpad;
        if (dpad != dpad_prev) {
            if(trim_mode){
                if (dpad & 0x01) pitch_trim += 0.1f;   // up
                if (dpad & 0x02) pitch_trim -= 0.1f;  // down
                if (dpad & 0x04) roll_trim += 0.1f;    // right
                if (dpad & 0x08) roll_trim -= 0.1f;   // left

                // printf("Trim adj: roll=%.1f pitch=%.1f\n", roll_trim, pitch_trim);
            } else {
            float adj = 0;
            if (dpad & 0x01) adj = 0.001f;
            if (dpad & 0x02) adj = -0.001f;
            if (dpad & 0x04) adj = 0.01f;
            if (dpad & 0x08) adj = -0.01f;

            if (adj != 0 && active) {
                if (pid_param == 0) { active->Kp += adj; if (active->Kp < 0) active->Kp = 0; }
                else if (pid_param == 1) { active->Kd += adj; if (active->Kd < 0) active->Kd = 0; }
                else { active->Ki += adj; if (active->Ki < 0) active->Ki = 0; }

                // printf("PID adj: axis=%d param=%d delta=%.2f\n", pid_axis, pid_param, adj);
            }
        }
        }
        dpad_prev = dpad;

        // Send telemetry back
        // txTelemetry.roll_kp = roll_pid->Kp;
        // txTelemetry.roll_kd = roll_pid->Kd;
        // txTelemetry.roll_ki = roll_pid->Ki;
        // txTelemetry.pitch_kp = pitch_pid->Kp;
        // txTelemetry.pitch_kd = pitch_pid->Kd;
        // txTelemetry.pitch_ki = pitch_pid->Ki;
        // txTelemetry.yaw_kp = yaw_pid->Kp;
        // txTelemetry.yaw_kd = yaw_pid->Kd;
        // txTelemetry.yaw_ki = yaw_pid->Ki;

        txTelemetry.roll_kp = use_rate_loop ? roll_rate->Kp : roll_angle->Kp;
        txTelemetry.roll_kd = use_rate_loop ? roll_rate->Kd : roll_angle->Kd;
        txTelemetry.roll_ki = use_rate_loop ? roll_rate->Ki : roll_angle->Ki;

        txTelemetry.pitch_kp = use_rate_loop ? pitch_rate->Kp : pitch_angle->Kp;
        txTelemetry.pitch_kd = use_rate_loop ? pitch_rate->Kd : pitch_angle->Kd;
        txTelemetry.pitch_ki = use_rate_loop ? pitch_rate->Ki : pitch_angle->Ki;

        txTelemetry.roll = roll;
        txTelemetry.pitch = pitch;
        txTelemetry.yaw = yaw;
        
        txTelemetry.roll_sp = roll_angle->sp;
        txTelemetry.pitch_sp = pitch_angle->sp;
        txTelemetry.yaw_rate_sp = yaw_rate->sp;

        txTelemetry.pid_axis = pid_axis;
        txTelemetry.pid_param = pid_param;
        // txTelemetry.pid_loop = use_rate_loop;
        txTelemetry.pid_loop = trim_mode ? 2 : (use_rate_loop ? 1 : 0);
                
        txTelemetry.roll_trim = roll_trim;
        txTelemetry.pitch_trim = pitch_trim;
        
        // if(rxPacket.transmitter_connection){
        //     txTelemetry.receiver_connection = true;
        // } else {
        //     txTelemetry.receiver_connection = false;
        // }
        
        esp_now_send(controllerMAC, (uint8_t *)&txTelemetry, sizeof(txTelemetry));
    }   
}
