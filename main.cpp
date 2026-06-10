#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>

//gcc -o week1 week_1_student.cpp -lwiringPi  -lm
// forward declaratiosns:
int setup_imu();
void calibrate_imu();      
void read_imu(float *out);    
void update_filter();
void setup_joystick();
void safety_check();
void trap(int signal);
void set_motors(int motor0, int motor1, int motor2, int motor3);
void camera_control(float current_yaw);
void calculate_motors();
void motor_enable();
//add global variable
int motor_address,accel_address,gyro_address;

//global variables
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float accel_x_calibration=0;
float accel_y_calibration=0;
float imu_data[8]; //accel xyz,  gyro xyz, roll, pitch
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle_a=0;
float roll_angle_a=0;
float temp[10];
float temps[1000][10];
float avgs[10];
float roll_t = 0;
float roll_gyro_delta;
float intl_pitch;
float intl_roll;
float pitch_gyro_delta;
float pitch_t = 0;
float camera_yaw;
// camera Y control
float camera_y_estimated = 0.0;
float camera_y_prev = 0.0;
float pitch_desired_camera = 0.0;
int last_cam_sequence = -1;
long last_cam_time = 0;
// camera X control
float camera_x_estimated = 0.0;
float camera_x_prev = 0.0;
float roll_desired_camera = 0.0;
int last_cam_sequence_x = -1;
long last_cam_time_x = 0;
// cam Z contrl
float camera_z_estimated = 0.0;
float camera_z_prev = 0.0;
float auto_thrust = 0.0;
float auto_thrust_integral = 0.0;
int last_cam_sequence_z = -1;
long last_cam_time_z = 0;

// safety check vars
#define A 0.01
#define GYRO_LIMIT 900
#define ANGLE_LIMIT 100
#define TIMEOUT 650000000
int last_sequence_num = -1;
long last_sequence_time = 0;

// motor commands vars
int motors_paused = 0;
int autonomy_paused = 0;
int prev_key2 = 0;
int motor_commands[4];
int motor1;
int motor2;
int motor3;
int motor4;
int thrust;
int thrust_desired;
float pitch_desired;
float pid_integral_pitch =0;
float roll_desired;
float pid_integral_roll =0;
float yaw_desired;

#define thurst_min 0
#define thrust_max 600 // FOR CAM
#define thrust_neutral 1200 // FOR CAM
#define thrust_amplitude 1200
#define pitch_amplitude 8
#define roll_amplitude 8
#define p_gain -3.2 // was 6.4, then 5
#define d_gain -3 // was 2.4, then 2, 
#define i_gain -.5
#define i_saturation 100.0
#define yaw_amplitude 80
#define p_gain_yaw -3 // was -3

// camera gains
#define p_gain_camera 12 //was 5, 12
#define d_gain_camera 7.5 //was 27, 7.5
#define desired_y 0.0
#define p_gain_camera_x 9   // was 20, 9
#define d_gain_camera_x 6 //was 1.75, 6
#define camera_yaw_gain 0.10
#define desired_x 0.0
#define p_gain_camera_z 1200 // was 150
#define d_gain_camera_z 400 // was -100
#define i_gain_camera_z 0 // was -2
#define z_auto_i_saturation 1200 // was 150
#define desired_z 1.5


//joystick integration 
struct Joystick // data struct from udp_rx file
{
  int key0;
  int key1;
  int key2;
  int key3;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  float x;
  float y;
  float z;
  float camera_yaw;
  int success;
  int sequence_num;
  
};
// joystick global variables
Joystick* shared_memory; 
volatile int run_program=1;
Joystick joystick_data; 

int main (int argc, char *argv[])
{
    setup_imu();
    calibrate_imu();
    time_prev = te.tv_nsec;
    setup_joystick();
    signal(SIGINT, trap);
    motor_enable();
    while(run_program == 1)
    {
      //printf("loop iteration\n");
      joystick_data = *shared_memory;
      read_imu(temp); 
      update_filter();   
      safety_check();
      camera_control(camera_yaw);
      calculate_motors();
      set_motors(motor1, motor2, motor3, motor4);
    }
    return(0);
}

void calibrate_imu()
{
 for (int i = 0; i < 1000; i++) {
  read_imu(temps[i]);
 }

  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < 1000; i++) {
      avgs[j] += temps[i][j];
    }
    avgs[j]/=1000.0f;
  }

  accel_x_calibration= avgs[0];
  accel_y_calibration= avgs[1];
  accel_z_calibration= avgs[2];
  x_gyro_calibration= avgs[3];
  y_gyro_calibration= avgs[4];
  z_gyro_calibration= avgs[5];
  roll_calibration= avgs[6];
  pitch_calibration= avgs[7];
  
  printf("calibrate, %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f \n\r", accel_x_calibration, accel_y_calibration, accel_z_calibration, x_gyro_calibration,y_gyro_calibration,z_gyro_calibration, roll_calibration, pitch_calibration);
  intl_pitch = 0.0;
  intl_roll = 0.0;
  sleep(5);

}

void read_imu(float *out) // input pointer to array adn write to out
{
  uint8_t address=0x12;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh=0;
  int vl=0;
  int vw=0;

  //accel reads

  address=0x12;//use 0x00 format for hex 
  vw=wiringPiI2CReadReg16(accel_address,address);    
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]=3.0*vw/32768.0;//convert to g's  


  address=0x14;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]=3.0*vw/32768.0;//convert to g's  

  
  address=0x16;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(accel_address,address);   
  //convert from 2's complement     
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=3.0*vw/32768.0;//convert to g's  


  //gyro reads

  address=0x02; //use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement          
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=1000.0*vw/32768.0;//convert to degrees/sec
  
  address=0x04;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);    
  //convert from 2's complement              
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=-1000.0*vw/32768.0;//convert to degrees/sec
  
  address=0x06;//use 0x00 format for hex
  vw=wiringPiI2CReadReg16(gyro_address,address);   
  //convert from 2's complement               
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=-1000.0*vw/32768.0;//convert to degrees/sec  
  
  // roll and pitch from accellerometer 
  pitch_angle_a = atan2(imu_data[1],imu_data[0])*180/M_PI;
  roll_angle_a = atan2(imu_data[0],imu_data[2])*180/M_PI;
  imu_data[6] = roll_angle_a;
  imu_data[7] = pitch_angle_a;

  //calibration
  imu_data[0] = imu_data[0] - accel_x_calibration; // calibrated x accelleration
  imu_data[1] = imu_data[1] - accel_y_calibration; // calibrated y accelleration
  imu_data[2] = imu_data[2] - accel_z_calibration; // calibrated z accelleration
  imu_data[3] = imu_data[3] - x_gyro_calibration; // x-gyro calibration
  imu_data[4] = imu_data[4] - y_gyro_calibration; // y-gyro calibration
  imu_data[5] = imu_data[5] - z_gyro_calibration; // z-gyro calibration
  imu_data[6] = imu_data[6] - roll_calibration; // calibrated roll angle
  imu_data[7] = imu_data[7] - pitch_calibration; // calibrated pitch angle
  
  out[0] = imu_data[0];
  out[1] = imu_data[1];
  out[2] = imu_data[2];
  out[3] = imu_data[3];
  out[4] = imu_data[4]; // roll gyro
  out[5] = imu_data[5];
  out[6] = imu_data[6];
  out[7] = imu_data[7];
  
}

void update_filter()
{
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  
  //comp. filter for roll, pitch here: 
  // roll and pitch from gyro
  roll_gyro_delta = temp[4]; // y comp
  intl_roll = intl_roll + (roll_gyro_delta*imu_diff);
  roll_t = (temp[6]*A) + ((1-A)*((roll_gyro_delta*imu_diff)+roll_t));
  temp[8] = roll_t;

  pitch_gyro_delta = temp[5]; // x comp
  intl_pitch = intl_pitch + (pitch_gyro_delta*imu_diff); 
  pitch_t = (temp[7]*A) + ((1-A)*((pitch_gyro_delta*imu_diff)+pitch_t)); // filter
  temp[9] = pitch_t;
  
  //print
  //printf("print IMU, %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n\r",temp[0],temp[1],temp[2],temp[3],temp[4],temp[5], temp[6], temp[7], roll_t, intl_roll, pitch_t, intl_pitch);
   
}

int setup_imu()
{
  wiringPiSetup ();
  motor_address=wiringPiI2CSetup (0x56) ; 
  
  //setup imu on I2C
  accel_address=wiringPiI2CSetup (0x19) ; 
  gyro_address=wiringPiI2CSetup (0x69) ; 
  
  if(accel_address==-1)
  {
    printf("-----cant connect to accel I2C device %d --------\n",accel_address);
    return -1;
  }
  else if(gyro_address==-1)
  {
    printf("-----cant connect to gyro I2C device %d --------\n",gyro_address);
    return -1;
  }
  else
  {
    printf("all i2c devices detected\n");
    sleep(1);
    wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); //power on accel    
    wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +_3g    
    wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel
    int filter_in_use = wiringPiI2CReadReg16(accel_address,0x40);
    printf("%X\n", filter_in_use);
    wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00); //power on gyro
    wiringPiI2CWriteReg8(gyro_address, 0x0f, 0x01); //set gyro to +-1000dps
    wiringPiI2CWriteReg8(gyro_address, 0x01, 0x03); //set data rate and bandwith
    
    
    sleep(1);
  }
  return 0;
}

//function to add for joystick integration
void setup_joystick()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Joystick*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  = shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}

void safety_check()
{
  // x gyro, y gyro or z gyro canno exceed 300 degrees/sec
    if(temp[3] > GYRO_LIMIT || temp[3] < -GYRO_LIMIT)
    {
    printf("x gyro too fast\n");
    motor1= 0;
    motor2= 0;
    motor3= 0;
    motor4= 0;
    run_program = 0;
    }
    if(temp[4] > GYRO_LIMIT || temp[4] < -GYRO_LIMIT)
    {
    printf("y gyro too fast\n");
    motor1= 0;
    motor2= 0;
    motor3= 0;
    motor4= 0;
    run_program = 0;
    }
    if(temp[5] > GYRO_LIMIT || temp[5] < -GYRO_LIMIT)
    {
    printf("z gyro too fast\n");
    motor1= 0;
    motor2= 0;
    motor3= 0;
    motor4= 0;
    run_program = 0;
    }
    if(roll_t > ANGLE_LIMIT || roll_t < -ANGLE_LIMIT)
    {
    printf("too much roll\n");
    motor1= 0;
    motor2= 0;
    motor3= 0;
    motor4= 0;
    run_program = 0;
    }
    if(pitch_t > ANGLE_LIMIT || pitch_t < -ANGLE_LIMIT)
    {
    printf("too much pitch\n");
    motor1= 0;
    motor2= 0;
    motor3= 0;
    motor4= 0;
    run_program = 0;
    }
    if(joystick_data.key1 == 1)
    {
    printf("B button pressed\n");
    motor1= 0;
    motor2= 0;
    motor3= 0;
    motor4= 0;
    run_program = 0;
    }
    
    // A button = PAUSE motors
    if(joystick_data.key0 == 1)
    {
      motors_paused = 1;
      printf("A pressed — motors PAUSED\n");
    }
    // Y button = UNPAUSE motors
    if(joystick_data.key3 == 1)
    {
      motors_paused = 0;
      printf("Y pressed — motors UNPAUSED\n");
    }
    // X button = autonomy off
    if(joystick_data.key2 == 1 && prev_key2 == 0)
    {
      autonomy_paused = !autonomy_paused;
      printf("X pressed — autonomy %s\n", autonomy_paused ? "paused" : "active");
    }
    prev_key2 = joystick_data.key2;
    
    ////////// joystick timeout
    // joystick alive
    if(joystick_data.sequence_num != last_sequence_num)
    { 
      last_sequence_num = joystick_data.sequence_num;
      last_sequence_time = te.tv_nsec;
    }
    // joystick dead
    else 
    {
      if(te.tv_nsec - last_sequence_time > TIMEOUT) 
      {
        printf("joystick timeout\n");
        run_program = 0;
      }
    }
    
}
void trap(int signal) 
{
printf("control + C pressed\n");
run_program = 0;
}
// CAMERA CONTROL
void camera_control(float camera_yaw) {
  if (autonomy_paused) 
  {
    yaw_desired = -((joystick_data.yaw-128)/128.0 * yaw_amplitude);
  }
  else 
  {

    camera_yaw = joystick_data.camera_yaw;
    yaw_desired = 0 + ((camera_yaw * camera_yaw_gain)*yaw_amplitude);
    
    // Y control — only run when new camera data arrives
    if (joystick_data.success == 1 && joystick_data.sequence_num != last_cam_sequence)
    {
      float camera_y = joystick_data.y;
      // exponential filter
      camera_y_estimated = camera_y_estimated * 0.6f + camera_y * 0.4f;

      // time  since last camera update
      float dt = (float)(te.tv_nsec - last_cam_time) / 1000000000.0f;
      if (dt <= 0.0f || dt > 1.0f) dt = 0.033f;  // fallback ~30fps

      // PD controller
      float y_error = camera_y_estimated - desired_y;
      float y_deriv = (camera_y_estimated - camera_y_prev) / dt;
      pitch_desired_camera = p_gain_camera * y_error + d_gain_camera * y_deriv;
      printf ("cam_y: %.3f pitch_cmd: %.3f\n", camera_y_estimated, pitch_desired_camera);

      camera_y_prev = camera_y_estimated;
      last_cam_sequence = joystick_data.sequence_num;
      last_cam_time = te.tv_nsec;
    }
    // X control — only run when new camera data arrives
    if (joystick_data.success == 1 && joystick_data.sequence_num != last_cam_sequence_x)
    {
      float camera_x = joystick_data.x;
      // exponential filter
      camera_x_estimated = camera_x_estimated * 0.6f + camera_x * 0.4f;

      // time  since last camera update
      float dt = (float)(te.tv_nsec - last_cam_time_x) / 1000000000.0f;
      if (dt <= 0.0f || dt > 1.0f) dt = 0.033f;  // fallback ~30fps

      // PD controller
      float x_error = camera_x_estimated - desired_x;
      float x_deriv = (camera_x_estimated - camera_x_prev) / dt;
      roll_desired_camera = p_gain_camera_x * x_error + d_gain_camera_x * x_deriv;

      camera_x_prev = camera_x_estimated;
      last_cam_sequence_x = joystick_data.sequence_num;
      last_cam_time_x = te.tv_nsec;
    }
    if (joystick_data.success == 1 && joystick_data.sequence_num != last_cam_sequence_z)
    {
      float camera_z = joystick_data.z;
      // exponential filter
      camera_z_estimated = camera_z_estimated * 0.6f + camera_z * 0.4f;

      // time since last camera update
      float dt = (float)(te.tv_nsec - last_cam_time_z) / 1000000000.0f;
      if (dt <= 0.0f || dt > 1.0f) dt = 0.033f;  // fallback ~30fps

      //PID controller
      float z_error = camera_z_estimated - desired_z;
      float z_deriv = (camera_z_estimated - camera_z_prev) / dt;

      if (autonomy_paused) {
        auto_thrust_integral = 0.0f;
      } else {
        auto_thrust_integral += i_gain_camera_z * z_error * dt;
        auto_thrust_integral = fmaxf(fminf(auto_thrust_integral, z_auto_i_saturation), -z_auto_i_saturation);
      }

      //auto_thrust_integral += i_gain_camera_z * z_error * dt;
      //auto_thrust_integral = fmaxf(fminf(auto_thrust_integral, z_auto_i_saturation), -z_auto_i_saturation);

      auto_thrust = p_gain_camera_z * z_error + d_gain_camera_z * z_deriv + auto_thrust_integral;

      printf("cam_z: %.3f auto_thrust: %.3f\n", camera_z_estimated, auto_thrust);

      camera_z_prev = camera_z_estimated;
      last_cam_sequence_z = joystick_data.sequence_num;
      last_cam_time_z = te.tv_nsec;
    }
  }
}

void calculate_motors()
{
  //printf("set motors called\n");
  float thrust_desired = (((joystick_data.thrust-128)/128.0) * thrust_amplitude);
  float thrust_joystick = thrust_neutral - thrust_desired;
  float thrust;
  if (autonomy_paused)
  {
    thrust = thrust_joystick;
  }
  else
  {
    thrust = 0.5f * thrust_joystick + 0.5f * (thrust_neutral + auto_thrust);
  }

  float pitch_desired_joystick = ((joystick_data.pitch-128)/128.0 * pitch_amplitude);

  if (autonomy_paused)
  {
    pitch_desired = pitch_desired_joystick;
  }
  else
  {
    pitch_desired = pitch_desired_joystick * 0.0f - pitch_desired_camera * 1.0f;
  }

  //printf("DEBUG raw_joy=%d  pitch_desired=%f\n", joystick_data.pitch, pitch_desired);
  float pitch_error = pitch_desired - pitch_t;

  float roll_desired_joystick = -((joystick_data.roll-128)/128.0 * roll_amplitude);

  if (autonomy_paused)
  {
    roll_desired = roll_desired_joystick;
  }
  else
  {
    roll_desired = roll_desired_joystick * 0.0f + roll_desired_camera * 1.0f;
  }

  float roll_error = roll_desired - roll_t;


  float yaw_error = yaw_desired - temp[3];   // temp[5] = z-gyro = yaw rate

  pid_integral_pitch = fmaxf(fminf((i_gain * pitch_error)+pid_integral_pitch, i_saturation), -i_saturation);
  pid_integral_roll = fmaxf(fminf((i_gain * roll_error)+pid_integral_roll, i_saturation), -i_saturation);
  //printf ("%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f\n", motor1, motor2, motor3, motor4, thrust, pitch_desired, pitch_t);
  if (motors_paused) 
  {
    motor1=20;
    motor2=20;
    motor3=20;
    motor4=20;
  } else {
  // front motors
  motor1 = thrust + p_gain * pitch_error - d_gain * pitch_gyro_delta + pid_integral_pitch - p_gain * roll_error + d_gain * roll_gyro_delta - pid_integral_roll + p_gain_yaw * yaw_error;
  motor3 = thrust + p_gain * pitch_error - d_gain * pitch_gyro_delta + pid_integral_pitch + p_gain * roll_error - d_gain * roll_gyro_delta + pid_integral_roll - p_gain_yaw * yaw_error;
  // // back motors
  motor2 = thrust - p_gain * pitch_error + d_gain * pitch_gyro_delta - pid_integral_pitch - p_gain * roll_error + d_gain * roll_gyro_delta - pid_integral_roll - p_gain_yaw * yaw_error;
  motor4 = thrust - p_gain * pitch_error + d_gain * pitch_gyro_delta - pid_integral_pitch + p_gain * roll_error - d_gain * roll_gyro_delta + pid_integral_roll + p_gain_yaw * yaw_error;
  }

  
  FILE *fs = fopen("/home/pi/0007.csv", "a");
  if (fs) {
    // fprintf(fs, "%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f\n",
    //          motor1, motor2, motor3, motor4,
    //          thrust, pitch_t, pitch_desired);
    fprintf(fs, "%10.5f,%10.5f\n", pitch_t, pitch_desired);
    //fprintf(fs, "%10.5f,%10.5f\n",
    //        // motor1, motor2, motor3, motor4,
    //        pitch_t, temp[7]);
    
    fclose(fs);
  }
  FILE *fs_roll = fopen("/home/pi/0007_roll.csv", "a");
  if (fs_roll) {
    fprintf(fs_roll, "%10.5f,%10.5f\n", roll_t, roll_desired);
    fclose(fs_roll);
  }

// log camera yaw: received angle from camera and resulting yaw command
  FILE *fs_cam = fopen("/home/pi/0009_camera.csv", "a");
  if (fs_cam) {
    fprintf(fs_cam, "%10.5f,%10.5f\n", joystick_data.camera_yaw, yaw_desired);
    fclose(fs_cam);
  }

  FILE *fs_yaw = fopen("/home/pi/0008.csv", "a");
  if (fs_yaw) 
  {
    fprintf(fs_yaw, "%10.5f,%10.5f,%10.5f,%10.5f,%10.5f,%10.5f\n",
          temp[7], pitch_gyro_delta, pitch_desired, pitch_t);
  fclose(fs_yaw);
  }
    // log thrust inputs: joystick desired thrust and camera auto thrust and camera z height
    static FILE *fs_thrust = NULL;
  if (fs_thrust == NULL) {
    fs_thrust = fopen("/home/pi/0010_thrust.csv", "w");
  }
  if (fs_thrust) {
    fprintf(fs_thrust, "%10.5f,%10.5f,%10.5f,%10.5f,%10.5f\n",
            thrust_joystick, auto_thrust, camera_z_estimated,
            camera_x_estimated, camera_y_estimated);
    fflush(fs_thrust);
  }



}

void set_motors(int motor0, int motor1, int motor2, int motor3)
{
  if(motor0<0)
    motor0=0;
  if(motor0>2000)
    motor0=2000;
  if(motor1<0)
    motor1=0;
  if(motor1>2000)
    motor1=2000;
  if(motor2<0)
    motor2=0;
  if(motor2>2000)
    motor2=2000;
  if(motor3<0)
    motor3=0;
  if(motor3>2000)
    motor3=2000;

  uint8_t motor_id=0;
  uint8_t special_command=0;
  uint16_t commanded_speed_0=1000;
  uint16_t commanded_speed_1=0;
  uint16_t commanded_speed=0;
  uint8_t data[2];

  // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
  //wiringPiI2CWrite (motor_address,data[0]) ;
  int com_delay=500;

  motor_id=0;
  commanded_speed=motor0;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  wiringPiI2CWrite(motor_address,data[0]);
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);
  motor_id=1;
  commanded_speed=motor1;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  wiringPiI2CWrite(motor_address,data[0]);
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);
  motor_id=2;
  commanded_speed=motor2;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  wiringPiI2CWrite(motor_address,data[0]);
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);
  motor_id=3;
  commanded_speed=motor3;
  data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
  data[1]=commanded_speed&0x7f;
  wiringPiI2CWrite(motor_address,data[0]);
  usleep(com_delay);
  wiringPiI2CWrite(motor_address,data[1]);
  usleep(com_delay);

  printf
  (
  "joystick: %d %d %d %d | yaw: %d pitch: %d roll: %d thrust: %d | "
  "x: %.3f y: %.3f z: %.3f yaw: %.3f | success: %d seq: %d\n",
  joystick_data.key0,
  joystick_data.key1,
  joystick_data.key2,
  joystick_data.key3,
  joystick_data.yaw,
  joystick_data.pitch,
  joystick_data.roll,
  joystick_data.thrust,
  joystick_data.x,
  joystick_data.y,
  joystick_data.z,
  joystick_data.camera_yaw,
  joystick_data.success,
  joystick_data.sequence_num,
  autonomy_paused ? "PAUSED" : "ACTIVE"
  );
}




void motor_enable()
{
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
    int cal_delay=50;
    
    for(int i=0;i<1000;i++)
    {
    
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }
     
    for(int i=0;i<2000;i++)
    {
    
      motor_id=0;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);

    }
    
     
    for(int i=0;i<500;i++)
    {
    
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);
    }
}