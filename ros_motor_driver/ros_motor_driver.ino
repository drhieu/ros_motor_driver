//Defines
#define relaypin  35
#define led  13

#define motor1_yel 21 //RR
#define motor2_yel 18 //MR 
#define motor3_yel 16 //TR
#define motor4_yel 41 //RL
#define motor5_yel 39 //ML
#define motor6_yel 24 //TL


//DIR
#define motor1_white 23 //RR
#define motor2_white 20 //MR
#define motor3_white 17 //TR
#define motor4_white 40 //RL
#define motor5_white 38 //ML
#define motor6_white 26 //TL

//SPD
#define motor1_blue 22 //RR
#define motor2_blue 19 //MR
#define motor3_blue 15 //TR
#define motor4_blue 14 //RL
#define motor5_blue 37 //ML
#define motor6_blue 25 //TL

// State machine defines
#define INIT_STATE    0x00
#define UPDATE_STATE  0X01
#define IDLE_STATE    0x02
#define BRAKE_STATE   0x04

// Global variables
static unsigned int flag = 0;
int right_motor;
int left_motor;
int start_byte = 253;
int mod_byte = 255;
int B = 0;
int right_motor_speed;
int left_motor_speed;


// struct
typedef struct robotmotion_s
{
    uint16_t robot_state;  
}   robotmotion_t;

static robotmotion_t state;

void setup() {
    Serial.begin(115200);
    pinMode(relaypin, OUTPUT);             
    pinMode(led, OUTPUT);             

   
  //DIR
    pinMode(motor1_white, OUTPUT);
    pinMode(motor2_white, OUTPUT);
    pinMode(motor3_white, OUTPUT);
    pinMode(motor4_white, OUTPUT);
    pinMode(motor5_white, OUTPUT);
    pinMode(motor6_white, OUTPUT);
  
    //SPD
    pinMode(motor1_blue, OUTPUT);
    pinMode(motor2_blue, OUTPUT);
    pinMode(motor3_blue, OUTPUT);
    pinMode(motor4_blue, OUTPUT);
    pinMode(motor5_blue, OUTPUT);
    pinMode(motor6_blue, OUTPUT);
  
    //
    analogWrite( motor1_blue, 0);
    analogWrite( motor2_blue, 0);
    analogWrite( motor3_blue, 0);
    analogWrite( motor4_blue, 0);
    analogWrite( motor5_blue, 0);
    analogWrite( motor6_blue, 0);
    
    // INput
    pinMode(motor1_yel, INPUT);
    pinMode(motor2_yel, INPUT);
    pinMode(motor3_yel, INPUT);
    pinMode(motor4_yel, INPUT);
    pinMode(motor5_yel, INPUT);
    pinMode(motor6_yel, INPUT);

    state.robot_state = INIT_STATE;
    digitalWrite(led, HIGH);
}


//Function to move motors
void movemotors(int left, int right) {
    //Serial.print(abs(left));
    //Serial.print(abs(right));
    
  
  if (left > 0) {
    digitalWrite(motor4_white, HIGH);
    digitalWrite(motor5_white, HIGH);
    digitalWrite(motor6_white, HIGH);
  } else if (left < 0) {
    digitalWrite(motor4_white, LOW);
    digitalWrite(motor5_white, LOW);
    digitalWrite(motor6_white, LOW);
  }
  if (right > 0) {
    digitalWrite(motor1_white, LOW);
    digitalWrite(motor2_white, LOW);
    digitalWrite(motor3_white, LOW);
  } else if (right < 0) {
    digitalWrite(motor1_white, HIGH);
    digitalWrite(motor2_white, HIGH);
    digitalWrite(motor3_white, HIGH);
  }
  analogWrite(motor1_blue, abs(right));
  analogWrite(motor2_blue, abs(right));
  analogWrite(motor3_blue, abs(right));
  analogWrite(motor4_blue, abs(left));
  analogWrite(motor5_blue, abs(left));
  analogWrite(motor6_blue, abs(left));
}


//Function to update motor speed and direction
void updatemotors() {

  if (right_motor_speed != 0 || left_motor_speed != 0) {
    digitalWrite(relaypin, HIGH);
    movemotors(left_motor_speed, right_motor_speed);

  } else {
    movemotors(0, 0);
    digitalWrite(relaypin, LOW);
  }
}

// State machine
static void robot_motion_state()
{
    switch(state.robot_state)
    {
        case (UPDATE_STATE):
          {
            updatemotors();
            state.robot_state = IDLE_STATE;
          }  
          break; /*end case (UPDATE_STATE)*/
        case (IDLE_STATE):
          {
            right_motor_speed = (right_motor_speed >= 150) ? (right_motor_speed - 1): (right_motor_speed <= -150) ? (right_motor_speed + 1) : (right_motor_speed = 0);
            left_motor_speed = (left_motor_speed >= 150) ? (left_motor_speed - 1): (left_motor_speed <= -150) ? (left_motor_speed + 1) : (left_motor_speed = 0);
          
            updatemotors();
          
            delay(10);           
          }
          break; /*end case (IDLE_STATE)*/
        case (BRAKE_STATE):
          {

            right_motor_speed *= -1;// (right_motor_speed > 0) ? (right_motor_speed * -1) : (right_motor_speed < 0) ? (right_motor_speed) : 0; 
            left_motor_speed *= -1;// (left_motor_speed > 0) ? (left_motor_speed  * -1) : (left_motor_speed < 0) ? (left_motor_speed) : 0; 

            updatemotors();

            delay(80);
                  
            do {

                  //right_motor_speed *= -1;
                 // left_motor_speed  *= -1;

                  //updatemotors();

                  right_motor_speed = (right_motor_speed >= 150) ? (right_motor_speed - 50): (right_motor_speed <= -150) ? (right_motor_speed + 50) : (right_motor_speed = 0);
                  left_motor_speed = (left_motor_speed >= 150) ? (left_motor_speed - 50): (left_motor_speed <= -150) ? (left_motor_speed + 50) : (left_motor_speed = 0);

                  updatemotors();
                  delay(200);
                  
            } while ((B == 1));// && (right_motor_speed != 0 || left_motor_speed !=0));

            
            if (state.robot_state == BRAKE_STATE)
            {
              state.robot_state = IDLE_STATE;
            }
          }
          break; /*end case (BRAKE_STATE)*/
        case (INIT_STATE):
        default:
          break;
    }
}

void loop() {

  robot_motion_state();
}



//Serial event
void serialEvent() {

  String motor_speed = "";
  String staRt;
  String left;
  String right;
  String cheCk;  
  String bRakes;
   
  while (Serial.available()) {
   

    char inChar = char(Serial.read());
    
    if (inChar == '\n')
    {
        unsigned int StringLength = motor_speed.length(); 
        
        unsigned int a = motor_speed.indexOf('@');
        unsigned int b = motor_speed.indexOf('#');
        unsigned int c = motor_speed.indexOf('&');  
        unsigned int d = motor_speed.indexOf('^');  

    
        staRt = motor_speed.substring(0, a);
        left = motor_speed.substring(a+1, b);
        right = motor_speed.substring(b+1, c);
        bRakes = motor_speed.substring(c+1, d);
        cheCk = motor_speed.substring(d+1, StringLength);


//        Serial.println(staRt);
//        Serial.println(left);
//        Serial.println(right);
//        Serial.println(cheCk);
    
        int checksum = (((start_byte - (left.toInt() + right.toInt())) % mod_byte) + mod_byte) % mod_byte;
//        Serial.println(checksum);
    
        if (staRt.toInt() == start_byte && checksum == cheCk.toInt() && bRakes.toInt() == 0) {
          
            state.robot_state = UPDATE_STATE;
            
            left_motor = left.toInt();
            right_motor = right.toInt();
            B = 0;

            // 0 = rotating backward 127 = stop 254 = rotating forward
            // converting to correct scale
            // convert anything in between 0 to 127 into -254 to 0
            // convert anything in between 127 to 254 into 0 to 254 
             
            left_motor_speed  = (!(left_motor == 127) * (52.0 / 63.0) * left_motor) + ((left_motor < 127) * -254) + ((left_motor > 127) * (2794.0 / 63.0));
            right_motor_speed = (!(right_motor == 127) * (52.0 / 63.0) * right_motor) + ((right_motor < 127) * -254) + ((right_motor > 127) * (2794.0 / 63.0));
            
            motor_speed = "";
    
//            Serial.println(left_motor);
//            Serial.println(right_motor);
    
        } 
        else if (bRakes.toInt() == 1)
        {
          if (left_motor_speed != 0 || right_motor_speed !=0)
          {
            state.robot_state = BRAKE_STATE;
            B = bRakes.toInt();
          }
          else
          {
            state.robot_state = INIT_STATE;
          }
        }
        else
        {
            motor_speed = "";
        }
    }
    else
    {
        motor_speed += inChar;  
    }
  }
}