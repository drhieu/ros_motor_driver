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


// Global variables
static unsigned int flag = 0;
int right_motor;
int left_motor;
int pos = 0;
int start_byte = 253;
int mod_byte = 255;
bool transmission_complete = false;

const long updatePeriod = 1000L; //ms
long lastUpdateTime = 0;
int pulseCount1 = 0;
long pulseCount2 = 0;

//int left_motor;
//int right_motor;


//Dead zone
const int deadzone = 20;


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
}


//Function to move motors
void movemotors(int left, int right) {
    //Serial.print(abs(left));
    //Serial.print(abs(right));
    
  //left_motor = left;
  //right_motor = right;
  
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



  // 0 = rotating backward 127 = stop 254 = rotating forward
  // converting to correct scale
  // convert anything in between 0 to 127 into -254 to 0
  // convert anything in between 127 to 254 into 0 to 254
    int left_motor_speed = (left_motor - 127) * 2;
    int right_motor_speed = (right_motor - 127) * 2;


  
//  int right_motor_speed = map(right_motor, 0, 255, -255, 255);
//  Serial.print(right_motor_speed);
//  int left_motor_speed = map(left_motor, 0, 255, -255, 255);
//  Serial.print(left_motor_speed);

  if (abs(right_motor_speed) <= deadzone) {
    right_motor_speed = 0;
  }
  if (abs(left_motor_speed) <= deadzone) {
    left_motor_speed = 0;
  }
  
  
  if (right_motor_speed != 0 || left_motor_speed != 0) {
    digitalWrite(relaypin, HIGH);
    movemotors(left_motor_speed, right_motor_speed);
  } else {
    movemotors(0, 0);
    digitalWrite(relaypin, LOW);
  }
}

void loop() {
  if(transmission_complete)
  {
    updatemotors();
    transmission_complete = false;
  }
  digitalWrite(led, HIGH);
  //updateRPM();
}



//Serial event

//void serialEvent()
//{
//  while (Serial.available())
//  {
//    static char left_motor_buffer[5];
//    static char right_motor_buffer[5];
//    static unsigned int index = 0;
//    char inChar = Serial.read();
//    switch(inChar)
//    {
//       case 'x':
//       /*start of buffer*/
//          //Serial.print(inChar);
//          flag = 1;
//          break;
//       case 'y':
//       /*End of buffer*/
//              left_motor_buffer[index] = '\0';
//              left_motor = atoi(left_motor_buffer);
//              //Serial.print(left_motor);
//              index = 0;
//              flag = 0;
//              transmission_complete = true;
//          break;
//       case 'z':
//       /*Middle of buffer*/
//              right_motor_buffer[index] = '\0';
//              right_motor = atoi(right_motor_buffer);
//              //Serial.print(right_motor);
//              index = 0;
//              flag = 2;
//          break;
//       default:
//          if(flag == 1)
//          {
//            /* Linear speed setup*/
//           right_motor_buffer[index++] = inChar;
//          }
//          else if(flag == 2)
//          {
//            /* Angular velocity setup*/
//            left_motor_buffer[index++] = inChar;
//          }
//          break;
//    }
//  }
//}

void serialEvent() {

  String motor_speed = "";
  String staRt;
  String left;
  String right;
  String cheCk;  
   
  while (Serial.available()) {
   

    char inChar = char(Serial.read());
    
    if (inChar == '\n')
    {
        unsigned int StringLength = motor_speed.length(); 
        
        unsigned int a = motor_speed.indexOf('@');
        unsigned int b = motor_speed.indexOf('#');
        unsigned int c = motor_speed.indexOf('&');  
    
        staRt = motor_speed.substring(0, a);
        left = motor_speed.substring(a+1, b);
        right = motor_speed.substring(b+1, c);
        cheCk = motor_speed.substring(c+1, StringLength);
    
//        Serial.println(staRt);
//        Serial.println(left);
//        Serial.println(right);
//        Serial.println(cheCk);
    
        int checksum = (((start_byte - (left.toInt() + right.toInt())) % mod_byte) + mod_byte) % mod_byte;
//        Serial.println(checksum);
    
        if (staRt.toInt() == start_byte && checksum == cheCk.toInt()) {
          
            transmission_complete = true;
            left_motor = left.toInt();
            right_motor = right.toInt();
            motor_speed = "";
    
//            Serial.println(left_motor);
//            Serial.println(right_motor);
    
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

//void pin_interrupts(){
//    attachInterrupt(digitalPinToInterrupt(motor2_yel),Update_encR1,RISING);
//    attachInterrupt(digitalPinToInterrupt(motor5_yel),Update_encR2,RISING);
//}
//
//void Update_encR1(){
//     pulseCount1++;
//}
//void Update_encR2(){
//     pulseCount2++;
//}
//
//void updateRPM()
//{
//   char msg1[256];
//   char msg2[256];
//   uint8_t dirL;  // Positive: 0    Negative: 1
//   uint8_t dirR;  // Positive: 0    Negative: 1
//   const uint8_t motor_positionL = 1;    // Right: 0  Left: 1
//   const uint8_t motor_positionR = 0;    // Right: 0  Left: 1
//   int l_motor = 1;
//   int r_motor = 1;
//
//
//   // Left encoder setup
//   if (l_motor > 0)
//     dirL = 0;
//   else
//     dirL = 1;
//     
//   pulseCount2 = 3000;
//   
//
//   //Right encoder setup
//   if (r_motor > 0)
//     dirR = 0;
//   else
//     dirR = 1;
//     
//   pulseCount1 = 6000;
//
//   sprintf(msg1, "%u%u%d\n", motor_positionL, dirL, pulseCount2); 
//   sprintf(msg2, "%u%u%d\n", motor_positionR, dirR, pulseCount1); 
//   Serial.print(msg1);
//   Serial.print(msg2);
//
//   pulseCount1 = 0;
//   pulseCount2 = 0;
//}
