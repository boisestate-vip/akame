
#ifndef ROBOCLAW_ACTUATOR_LIB

#define ROBOCLAW_ACTUATOR_LIB

#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/select.h>

#define ROBOCLAW_MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg 
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg
/* best effort, https://www.man7.org/linux/man-pages/man3/termios.3.html */
static speed_t get_baud_const(int baud) {
   static speed_t vals[] = {
      B0,
      B50,
      B75,
      B110,
      B134,
      B150,
      B200,
      B300,
      B600,
      B1200,
      B1800,
      B2400,
      B4800,
      B9600,
      B19200,
      B38400,
      B57600,
      B115200,
      B230400,
      B460800,
      B500000,
      B576000,
      B921600,
      B1000000,
      B1152000,
      B1500000,
      B2000000,
   };
   static int baud_nums[] = {
      0,
      50,
      75,
      110,
      134,
      150,
      200,
      300,
      600,
      1200,
      1800,
      2400,
      4800,
      9600,
      19200,
      38400,
      57600,
      115200,
      230400,
      460800,
      500000,
      576000,
      921600,
      1000000,
      1152000,
      1500000,
      2000000,
   };

   /* position ourselves in between the cloest values */
   int idx = 0;
   while (baud > baud_nums[idx] && idx < 26)
      idx += 1;

   /* crappy baud given */
   if (idx == 26 || idx == 0)
      return vals[idx];

   /* exact match */
   if (baud == baud_nums[idx])
      return vals[idx];

   int dist_low = std::abs(baud_nums[idx-1] - baud);
   int dist_high = std::abs(baud_nums[idx] - baud);

   /* closest value */
   if (dist_low > dist_high)
      return vals[idx];
   return vals[idx-1];
}

/* try and connect and set the given baud rate for the waveshare */
static int try_connect(const char * device, int baud) {

   int fd = open(device,O_WRONLY|O_NONBLOCK);
   if (fd < 0) {
      perror("open");
      return -1;
   }

   struct termios t;
   int res0 = tcgetattr(fd,&t);
   if (res0 < 0) {
      perror("tcgetattr");
      return -1;
   }

   int res1 = cfsetispeed(&t,get_baud_const(baud));
   if (res1 < 0) {
      perror("cfsetispeed");
      return -1;
   }

   int res2 = tcsetattr(fd,TCSANOW,&t);
   if (res2 < 0) {
      perror("tcsetattr");
      return -1;
   }

   return fd;
}

static uint32_t tnow(void) {
   struct timespec val;
   clock_gettime(CLOCK_REALTIME,&val);

   uint32_t micros = (val.tv_nsec/1000) + (val.tv_sec*1000000);

   return micros;
}

enum {
   ERROR_NONE			= 0x000000,
   ERROR_ESTOP			= 0x000001,	//Error: E-Stop active
   ERROR_TEMP			= 0x000002,	//Error: Temperature Sensor 1 >=100c
   ERROR_TEMP2			= 0x000004,	//Error: Temperature Sensor 2 >=100C (available only on some models)
   ERROR_MBATHIGH		= 0x000008,	//Error: Main Battery Over Voltage
   ERROR_LBATHIGH		= 0x000010,	//Error: Logic Battery High Voltage
   ERROR_LBATLOW		= 0x000020,	//Error: Logic Battery Low Voltage
   ERROR_FAULTM1		= 0x000040,	//Error: Motor 1 Driver Fault (only on some models)
   ERROR_FAULTM2		= 0x000080,	//Error: Motor 2 Driver Fault (only on some models)
   ERROR_SPEED1		= 0x000100,	//Error: Motor 1 Speed Error Limit
   ERROR_SPEED2		= 0x000200,	//Error: Motor 2 Speed Error Limit
   ERROR_POS1			= 0x000400,	//Error: Motor 1 Position Error Limit
   ERROR_POS2			= 0x000800,	//Error: MOtor2 Position Error Limit
   WARN_OVERCURRENTM1	= 0x010000, //Warning: Motor 1 Current Limited
   WARN_OVERCURRENTM2	= 0x020000, //Warning: Motor 2 CUrrent Limited
   WARN_MBATHIGH		= 0x040000, //Warning: Main Battery Voltage High
   WARN_MBATLOW		= 0x080000, //Warning: Main Battery Low Voltage
   WARN_TEMP			= 0x100000, //Warning: Temperaure Sensor 1 >=85C
   WARN_TEMP2			= 0x200000, //Warning: Temperature Sensor 2 >=85C (available only on some models)
   WARN_S4				= 0x400000, //Warning: Motor 1 Home/Limit Signal
   WARN_S5				= 0x800000, //Warning: Motor 2 Home/Limit Signal
};

enum {
   M1FORWARD = 0,
   M1BACKWARD = 1,
   SETMINMB = 2,
   SETMAXMB = 3,
   M2FORWARD = 4,
   M2BACKWARD = 5,
   M17BIT = 6,
   M27BIT = 7,
   MIXEDFORWARD = 8,
   MIXEDBACKWARD = 9,
   MIXEDRIGHT = 10,
   MIXEDLEFT = 11,
   MIXEDFB = 12,
   MIXEDLR = 13,
   GETM1ENC = 16,
   GETM2ENC = 17,
   GETM1SPEED = 18,
   GETM2SPEED = 19,
   RESETENC = 20,
   GETVERSION = 21,
   SETM1ENCCOUNT = 22,
   SETM2ENCCOUNT = 23,
   GETMBATT = 24,
   GETLBATT = 25,
   SETMINLB = 26,
   SETMAXLB = 27,
   SETM1PID = 28,
   SETM2PID = 29,
   GETM1ISPEED = 30,
   GETM2ISPEED = 31,
   M1DUTY = 32,
   M2DUTY = 33,
   MIXEDDUTY = 34,
   M1SPEED = 35,
   M2SPEED = 36,
   MIXEDSPEED = 37,
   M1SPEEDACCEL = 38,
   M2SPEEDACCEL = 39,
   MIXEDSPEEDACCEL = 40,
   M1SPEEDDIST = 41,
   M2SPEEDDIST = 42,
   MIXEDSPEEDDIST = 43,
   M1SPEEDACCELDIST = 44,
   M2SPEEDACCELDIST = 45,
   MIXEDSPEEDACCELDIST = 46,
   GETBUFFERS = 47,
   GETPWMS = 48,
   GETCURRENTS = 49,
   MIXEDSPEED2ACCEL = 50,
   MIXEDSPEED2ACCELDIST = 51,
   M1DUTYACCEL = 52,
   M2DUTYACCEL = 53,
   MIXEDDUTYACCEL = 54,
   READM1PID = 55,
   READM2PID = 56,
   SETMAINVOLTAGES = 57,
   SETLOGICVOLTAGES = 58,
   GETMINMAXMAINVOLTAGES = 59,
   GETMINMAXLOGICVOLTAGES = 60,
   SETM1POSPID = 61,
   SETM2POSPID = 62,
   READM1POSPID = 63,
   READM2POSPID = 64,
   M1SPEEDACCELDECCELPOS = 65,
   M2SPEEDACCELDECCELPOS = 66,
   MIXEDSPEEDACCELDECCELPOS = 67,
   SETM1DEFAULTACCEL = 68,
   SETM2DEFAULTACCEL = 69,
   SETPINFUNCTIONS = 74,
   GETPINFUNCTIONS = 75,
   SETDEADBAND	= 76,
   GETDEADBAND	= 77,
   GETENCODERS = 78,
   GETISPEEDS = 79,
   RESTOREDEFAULTS = 80,
   GETTEMP = 82,
   GETTEMP2 = 83,	//Only valid on some models
   GETERROR = 90,
   GETENCODERMODE = 91,
   SETM1ENCODERMODE = 92,
   SETM2ENCODERMODE = 93,
   WRITENVM = 94,
   READNVM = 95,	//Reloads values from Flash into Ram
   SETCONFIG = 98,
   GETCONFIG = 99,
   GETVOLTS = 100,
   GETTEMPS = 101,
   SETAUXDUTYS = 102,
   GETENCSTATUS = 103,
   GETAUXDUTYS = 104,
   SETAUTO1 = 105,
   SETAUTO2 = 106,
   GETAUTOS = 107,
   GETSPEEDS = 108,
   SETSPEEDERRORLIMIT = 109,
   GETSPEEDERRORLIMIT = 110,
   GETSPEEDERRORS = 111,
   SETPOSERRORLIMIT = 112,
   GETPOSERRORLIMIT = 113,
   GETPOSERRORS = 114,
   SETOFFSETS = 115,	//MCP only
   GETOFFSETS = 116,	//MCP only

   M1POS = 119,
   M2POS = 120,
   MIXEDPOS = 121,
   M1SPEEDPOS = 122,
   M2SPEEDPOS = 123,
   MIXEDSPEEDPOS = 124,
   M1PPOS = 125,
   M2PPOS = 126,
   MIXEDPPOS = 127,

   SETM1LR = 128, //MCP only
   SETM2LR = 129, //MCP only
   GETM1LR = 130, //MCP only
   GETM2LR = 131, //MCP only
   SETM1MAXCURRENT = 133,
   SETM2MAXCURRENT = 134,
   GETM1MAXCURRENT = 135,
   GETM2MAXCURRENT = 136,
   SETPWMMODE = 148,
   GETPWMMODE = 149,
   FLAGBOOTLOADER = 255
};	//Only available via USB communications

class RoboClaw {

private:

   uint16_t crc;
   uint32_t timeout;

   int fd;

private:

   size_t _write(uint8_t byte) {
      return write(fd,&byte,1);
   }

   int _read() {
      int buf;
      read(fd,&buf,1);
      return buf;
   }

   int available() {
      fd_set readfs;
      struct timeval delay = { 0, 0};
      FD_ZERO(&readfs);
      FD_SET(fd,&readfs);
      int live = select(fd+1,&readfs,NULL,NULL,&delay);
      return FD_ISSET(fd,&readfs);
   }

   int _read(uint32_t timeout) {
      uint32_t start = tnow();
      while (!available()) {
         if (tnow()-start >= timeout)
            return -1;
      }
      return _read();
   }

   void clear() {
      while (available()) {
         _read();
      }
   }

   void crc_clear() {
      crc = 0;
   }

   void crc_update (uint8_t data) {
      int i;
      crc = crc ^ ((uint16_t)data << 8);
      for (i=0; i<8; i++)
      {
         if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
         else
            crc <<= 1;
      }
   }

   uint16_t crc_get() {
      return crc;
   }

   bool write_n(int cnt, ... ) {
      uint8_t trys=ROBOCLAW_MAXRETRY;
      do{
         crc_clear();
         //send data with crc
         va_list marker;
         va_start( marker, cnt );     /* Initialize variable arguments. */
         for(uint8_t index=0;index<cnt;index++){
            uint8_t data = va_arg(marker, int);
            crc_update(data);
            _write(data);
         }
         va_end( marker );              /* Reset variable arguments.      */
         uint16_t crc = crc_get();
         _write(crc>>8);
         _write(crc);
         if(_read(timeout)==0xFF)
            return true;
      }while(trys--);
      return false;
   }

   /* maybe will need to fix this */
   void flush() {}

   bool read_n(int cnt, uint8_t address, int cmd, ...) {
      uint32_t value=0;
      uint8_t trys=ROBOCLAW_MAXRETRY;
      int16_t data;
      do{
         flush();

         data=0;
         crc_clear();
         _write(address);
         crc_update(address);
         _write(cmd);
         crc_update(cmd);

         //send data with crc
         va_list marker;
         va_start( marker, cmd );     /* Initialize variable arguments. */
         for(uint8_t index=0;index<cnt;index++){
            uint32_t *ptr = va_arg(marker, uint32_t *);

            if(data!=-1){
               data = _read(timeout);
               crc_update(data);
               value=(uint32_t)data<<24;
            }
            else{
               break;
            }

            if(data!=-1){
               data = _read(timeout);
               crc_update(data);
               value|=(uint32_t)data<<16;
            }
            else{
               break;
            }

            if(data!=-1){
               data = _read(timeout);
               crc_update(data);
               value|=(uint32_t)data<<8;
            }
            else{
               break;
            }

            if(data!=-1){
               data = _read(timeout);
               crc_update(data);
               value|=(uint32_t)data;
            }
            else{
               break;
            }

            *ptr = value;
         }
         va_end( marker );              /* Reset variable arguments.      */

         if(data!=-1){
            uint16_t ccrc;
            data = _read(timeout);
            if(data!=-1){
               ccrc = data << 8;
               data = _read(timeout);
               if(data!=-1){
                  ccrc |= data;
                  return crc_get()==ccrc;
               }
            }
         }
      }while(trys--);

      return false;
   }

   uint8_t Read1(uint8_t address,uint8_t cmd,bool *valid){
      uint8_t crc;

      if(valid)
         *valid = false;

      uint8_t value=0;
      uint8_t trys=ROBOCLAW_MAXRETRY;
      int16_t data;
      do{
         flush();

         crc_clear();
         _write(address);
         crc_update(address);
         _write(cmd);
         crc_update(cmd);

         data = _read(timeout);
         crc_update(data);
         value=data;

         if(data!=-1){
            uint16_t ccrc;
            data = _read(timeout);
            if(data!=-1){
               ccrc = data << 8;
               data = _read(timeout);
               if(data!=-1){
                  ccrc |= data;
                  if(crc_get()==ccrc){
                     if(valid)
                        *valid = true;
                     return value;
                  }
               }
            }
         }
      }while(trys--);

      return false;
   }

   uint16_t Read2(uint8_t address,uint8_t cmd,bool *valid){
      uint8_t crc;

      if(valid)
         *valid = false;

      uint16_t value=0;
      uint8_t trys=ROBOCLAW_MAXRETRY;
      int16_t data;
      do{
         flush();

         crc_clear();
         _write(address);
         crc_update(address);
         _write(cmd);
         crc_update(cmd);

         data = _read(timeout);
         crc_update(data);
         value=(uint16_t)data<<8;

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            value|=(uint16_t)data;
         }

         if(data!=-1){
            uint16_t ccrc;
            data = _read(timeout);
            if(data!=-1){
               ccrc = data << 8;
               data = _read(timeout);
               if(data!=-1){
                  ccrc |= data;
                  if(crc_get()==ccrc){
                     if(valid)
                        *valid = true;
                     return value;
                  }
               }
            }
         }
      }while(trys--);

      return false;
   }

   uint32_t Read4(uint8_t address, uint8_t cmd, bool *valid){
      uint8_t crc;

      if(valid)
         *valid = false;

      uint32_t value=0;
      uint8_t trys=ROBOCLAW_MAXRETRY;
      int16_t data;
      do{
         flush();

         crc_clear();
         _write(address);
         crc_update(address);
         _write(cmd);
         crc_update(cmd);

         data = _read(timeout);
         crc_update(data);
         value=(uint32_t)data<<24;

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            value|=(uint32_t)data<<16;
         }

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            value|=(uint32_t)data<<8;
         }

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            value|=(uint32_t)data;
         }

         if(data!=-1){
            uint16_t ccrc;
            data = _read(timeout);
            if(data!=-1){
               ccrc = data << 8;
               data = _read(timeout);
               if(data!=-1){
                  ccrc |= data;
                  if(crc_get()==ccrc){
                     if(valid)
                        *valid = true;
                     return value;
                  }
               }
            }
         }
      }while(trys--);

      return false;
   }

   uint32_t Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid){
      uint8_t crc;

      if(valid)
         *valid = false;

      uint32_t value=0;
      uint8_t trys=ROBOCLAW_MAXRETRY;
      int16_t data;
      do{
         flush();

         crc_clear();
         _write(address);
         crc_update(address);
         _write(cmd);
         crc_update(cmd);

         data = _read(timeout);
         crc_update(data);
         value=(uint32_t)data<<24;

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            value|=(uint32_t)data<<16;
         }

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            value|=(uint32_t)data<<8;
         }

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            value|=(uint32_t)data;
         }

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            if(status)
               *status = data;
         }

         if(data!=-1){
            uint16_t ccrc;
            data = _read(timeout);
            if(data!=-1){
               ccrc = data << 8;
               data = _read(timeout);
               if(data!=-1){
                  ccrc |= data;
                  if(crc_get()==ccrc){
                     if(valid)
                        *valid = true;
                     return value;
                  }
               }
            }
         }
      }while(trys--);

      return false;
   }


public:

   RoboClaw(const char * device, long baud, uint32_t tout) {
      this->timeout = tout;
      this->fd = try_connect(device,baud);
      if (this->fd == -1) {
         printf("failing!\n");
         exit(1);
      }
   }

   ~RoboClaw() {
   }

   bool BackwardM1(uint8_t address, uint8_t speed){
      return write_n(3,address,M1BACKWARD,speed);
   }

   bool SetMinVoltageMainBattery(uint8_t address, uint8_t voltage){
      return write_n(3,address,SETMINMB,voltage);
   }

   bool SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage){
      return write_n(3,address,SETMAXMB,voltage);
   }

   bool ForwardM2(uint8_t address, uint8_t speed){
      return write_n(3,address,M2FORWARD,speed);
   }

   bool BackwardM2(uint8_t address, uint8_t speed){
      return write_n(3,address,M2BACKWARD,speed);
   }

   bool ForwardBackwardM1(uint8_t address, uint8_t speed){
      return write_n(3,address,M17BIT,speed);
   }

   bool ForwardBackwardM2(uint8_t address, uint8_t speed){
      return write_n(3,address,M27BIT,speed);
   }

   bool ForwardMixed(uint8_t address, uint8_t speed){
      return write_n(3,address,MIXEDFORWARD,speed);
   }

   bool BackwardMixed(uint8_t address, uint8_t speed){
      return write_n(3,address,MIXEDBACKWARD,speed);
   }

   bool TurnRightMixed(uint8_t address, uint8_t speed){
      return write_n(3,address,MIXEDRIGHT,speed);
   }

   bool TurnLeftMixed(uint8_t address, uint8_t speed){
      return write_n(3,address,MIXEDLEFT,speed);
   }

   bool ForwardBackwardMixed(uint8_t address, uint8_t speed){
      return write_n(3,address,MIXEDFB,speed);
   }

   bool LeftRightMixed(uint8_t address, uint8_t speed){
      return write_n(3,address,MIXEDLR,speed);
   }

   uint32_t ReadEncM1(uint8_t address, uint8_t *status,bool *valid){
      return Read4_1(address,GETM1ENC,status,valid);
   }

   uint32_t ReadEncM2(uint8_t address, uint8_t *status,bool *valid){
      return Read4_1(address,GETM2ENC,status,valid);
   }

   uint32_t ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid){
      return Read4_1(address,GETM1SPEED,status,valid);
   }

   uint32_t ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid){
      return Read4_1(address,GETM2SPEED,status,valid);
   }

   bool ResetEncoders(uint8_t address){
      return write_n(2,address,RESETENC);
   }

   bool ReadVersion(uint8_t address,char *version){
      int16_t data;
      uint8_t trys=ROBOCLAW_MAXRETRY;
      do{
         flush();

         data = 0;

         crc_clear();
         _write(address);
         crc_update(address);
         _write(GETVERSION);
         crc_update(GETVERSION);

         uint8_t i;
         for(i=0;i<48;i++){
            if(data!=-1){
               data=_read(timeout);
               version[i] = data;
               crc_update(version[i]);
               if(version[i]==0){
                  uint16_t ccrc;
                  data = _read(timeout);
                  if(data!=-1){
                     ccrc = data << 8;
                     data = _read(timeout);
                     if(data!=-1){
                        ccrc |= data;
                        return crc_get()==ccrc;
                     }
                  }
                  break;
               }
            }
            else{
               break;
            }
         }
      }while(trys--);

      return false;
   }

   bool SetEncM1(uint8_t address, int32_t val){
      return write_n(6,address,SETM1ENCCOUNT,SetDWORDval(val));
   }

   bool SetEncM2(uint8_t address, int32_t val){
      return write_n(6,address,SETM2ENCCOUNT,SetDWORDval(val));
   }

   uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid){
      return Read2(address,GETMBATT,valid);
   }

   uint16_t ReadLogicBatteryVoltage(uint8_t address,bool *valid){
      return Read2(address,GETLBATT,valid);
   }

   bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage){
      return write_n(3,address,SETMINLB,voltage);
   }

   bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage){
      return write_n(3,address,SETMAXLB,voltage);
   }

   bool SetM1VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
      uint32_t kp = kp_fp*65536;
      uint32_t ki = ki_fp*65536;
      uint32_t kd = kd_fp*65536;
      return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
   }

   bool SetM2VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
      uint32_t kp = kp_fp*65536;
      uint32_t ki = ki_fp*65536;
      uint32_t kd = kd_fp*65536;
      return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
   }

   uint32_t ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid){
      return Read4_1(address,GETM1ISPEED,status,valid);
   }

   uint32_t ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid){
      return Read4_1(address,GETM2ISPEED,status,valid);
   }

   bool DutyM1(uint8_t address, uint16_t duty){
      return write_n(4,address,M1DUTY,SetWORDval(duty));
   }

   bool DutyM2(uint8_t address, uint16_t duty){
      return write_n(4,address,M2DUTY,SetWORDval(duty));
   }

   bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2){
      return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
   }

   bool SpeedM1(uint8_t address, uint32_t speed){
      return write_n(6,address,M1SPEED,SetDWORDval(speed));
   }

   bool SpeedM2(uint8_t address, uint32_t speed){
      return write_n(6,address,M2SPEED,SetDWORDval(speed));
   }

   bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2){
      return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
   }

   bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed){
      return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
   }

   bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed){
      return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
   }
   bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2){
      return write_n(14,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
   }

   bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
      return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
   }

   bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
      return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
   }

   bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
      return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
   }

   bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
      return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
   }

   bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
      return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
   }

   bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
      return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
   }

   bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2){
      bool valid;
      uint16_t value = Read2(address,GETBUFFERS,&valid);
      if(valid){
         depth1 = value>>8;
         depth2 = value;
      }
      return valid;
   }

   bool ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2){
      bool valid;
      uint32_t value = Read4(address,GETPWMS,&valid);
      if(valid){
         pwm1 = value>>16;
         pwm2 = value&0xFFFF;
      }
      return valid;
   }

   bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2){
      bool valid;
      uint32_t value = Read4(address,GETCURRENTS,&valid);
      if(valid){
         current1 = value>>16;
         current2 = value&0xFFFF;
      }
      return valid;
   }

   bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
      return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
   }

   bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
      return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
   }

   bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel){
      return write_n(8,address,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
   }

   bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel){
      return write_n(8,address,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
   }

   bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2){
      return write_n(14,address,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2));
   }

   bool ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
      uint32_t Kp,Ki,Kd;
      bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
      Kp_fp = ((float)Kp)/65536;
      Ki_fp = ((float)Ki)/65536;
      Kd_fp = ((float)Kd)/65536;
      return valid;
   }

   bool ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
      uint32_t Kp,Ki,Kd;
      bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
      Kp_fp = ((float)Kp)/65536;
      Ki_fp = ((float)Ki)/65536;
      Kd_fp = ((float)Kd)/65536;
      return valid;
   }

   bool SetMainVoltages(uint8_t address,uint16_t min,uint16_t max){
      return write_n(6,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
   }

   bool SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
      return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
   }

   bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max){
      bool valid;
      uint32_t value = Read4(address,GETMINMAXMAINVOLTAGES,&valid);
      if(valid){
         min = value>>16;
         max = value&0xFFFF;
      }
      return valid;
   }

   bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max){
      bool valid;
      uint32_t value = Read4(address,GETMINMAXLOGICVOLTAGES,&valid);
      if(valid){
         min = value>>16;
         max = value&0xFFFF;
      }
      return valid;
   }

   bool SetM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){			
      uint32_t kp=kp_fp*1024;
      uint32_t ki=ki_fp*1024;
      uint32_t kd=kd_fp*1024;
      return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
   }

   bool SetM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){			
      uint32_t kp=kp_fp*1024;
      uint32_t ki=ki_fp*1024;
      uint32_t kd=kd_fp*1024;
      return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
   }

   bool ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
      uint32_t Kp,Ki,Kd;
      bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
      Kp_fp = ((float)Kp)/1024;
      Ki_fp = ((float)Ki)/1024;
      Kd_fp = ((float)Kd)/1024;
      return valid;
   }

   bool ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
      uint32_t Kp,Ki,Kd;
      bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
      Kp_fp = ((float)Kp)/1024;
      Ki_fp = ((float)Ki)/1024;
      Kd_fp = ((float)Kd)/1024;
      return valid;
   }

   bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
      return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
   }

   bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
      return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
   }

   bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
      return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
   }

   bool SetM1DefaultAccel(uint8_t address, uint32_t accel){
      return write_n(6,address,SETM1DEFAULTACCEL,SetDWORDval(accel));
   }

   bool SetM2DefaultAccel(uint8_t address, uint32_t accel){
      return write_n(6,address,SETM2DEFAULTACCEL,SetDWORDval(accel));
   }

   bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode){
      return write_n(5,address,SETPINFUNCTIONS,S3mode,S4mode,S5mode);
   }

   bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode){
      uint8_t crc;
      bool valid = false;
      uint8_t val1,val2,val3;
      uint8_t trys=ROBOCLAW_MAXRETRY;
      int16_t data;
      do{
         flush();

         crc_clear();
         _write(address);
         crc_update(address);
         _write(GETPINFUNCTIONS);
         crc_update(GETPINFUNCTIONS);

         data = _read(timeout);
         crc_update(data);
         val1=data;

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            val2=data;
         }

         if(data!=-1){
            data = _read(timeout);
            crc_update(data);
            val3=data;
         }

         if(data!=-1){
            uint16_t ccrc;
            data = _read(timeout);
            if(data!=-1){
               ccrc = data << 8;
               data = _read(timeout);
               if(data!=-1){
                  ccrc |= data;
                  if(crc_get()==ccrc){
                     S3mode = val1;
                     S4mode = val2;
                     S5mode = val3;
                     return true;
                  }
               }
            }
         }
      }while(trys--);

      return false;
   }

   bool SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max){
      return write_n(4,address,SETDEADBAND,Min,Max);
   }

   bool GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max){
      bool valid;
      uint16_t value = Read2(address,GETDEADBAND,&valid);
      if(valid){
         Min = value>>8;
         Max = value;
      }
      return valid;
   }

   bool ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2){
      bool valid = read_n(2,address,GETENCODERS,&enc1,&enc2);
      return valid;
   }

   bool ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2){
      bool valid = read_n(2,address,GETISPEEDS,&ispeed1,&ispeed2);
      return valid;
   }

   bool RestoreDefaults(uint8_t address){
      return write_n(2,address,RESTOREDEFAULTS);
   }

   bool ReadTemp(uint8_t address, uint16_t &temp){
      bool valid;
      temp = Read2(address,GETTEMP,&valid);
      return valid;
   }

   bool ReadTemp2(uint8_t address, uint16_t &temp){
      bool valid;
      temp = Read2(address,GETTEMP2,&valid);
      return valid;
   }

   uint32_t ReadError(uint8_t address,bool *valid){
      return Read4(address,GETERROR,valid);
   }

   bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode){
      bool valid;
      uint16_t value = Read2(address,GETENCODERMODE,&valid);
      if(valid){
         M1mode = value>>8;
         M2mode = value;
      }
      return valid;
   }

   bool SetM1EncoderMode(uint8_t address,uint8_t mode){
      return write_n(3,address,SETM1ENCODERMODE,mode);
   }

   bool SetM2EncoderMode(uint8_t address,uint8_t mode){
      return write_n(3,address,SETM2ENCODERMODE,mode);
   }

   bool WriteNVM(uint8_t address){
      return write_n(6,address,WRITENVM, SetDWORDval(0xE22EAB7A) );
   }

   bool ReadNVM(uint8_t address){
      return write_n(2,address,READNVM);
   }

   bool SetConfig(uint8_t address, uint16_t config){
      return write_n(4,address,SETCONFIG,SetWORDval(config));
   }

   bool GetConfig(uint8_t address, uint16_t &config){
      bool valid;
      uint16_t value = Read2(address,GETCONFIG,&valid);
      if(valid){
         config = value;
      }
      return valid;
   }

   bool SetM1MaxCurrent(uint8_t address,uint32_t max){
      return write_n(10,address,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
   }

   bool SetM2MaxCurrent(uint8_t address,uint32_t max){
      return write_n(10,address,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
   }

   bool ReadM1MaxCurrent(uint8_t address,uint32_t &max){
      uint32_t tmax,dummy;
      bool valid = read_n(2,address,GETM1MAXCURRENT,&tmax,&dummy);
      if(valid)
         max = tmax;
      return valid;
   }

   bool ReadM2MaxCurrent(uint8_t address,uint32_t &max){
      uint32_t tmax,dummy;
      bool valid = read_n(2,address,GETM2MAXCURRENT,&tmax,&dummy);
      if(valid)
         max = tmax;
      return valid;
   }

   bool SetPWMMode(uint8_t address, uint8_t mode){
      return write_n(3,address,SETPWMMODE,mode);
   }

   bool GetPWMMode(uint8_t address, uint8_t &mode){
      bool valid;
      uint8_t value = Read1(address,GETPWMMODE,&valid);
      if(valid){
         mode = value;
      }
      return valid;
   }

/**
 * @brief Move Motor 1 to a percentage position
 *
 * Commands Motor 1 to move to a position specified as a percentage
 * of the configured position range.
 *
 * @param address Controller address
 * @param position Target position as a percentage (-32768 to +32767)
 * @param buffer Command buffer control:
 *        1 = Execute immediately
 *        0 = Add to buffer
 * @return true if successful, false otherwise
 */
   bool M1PercentPosition(uint8_t address, int16_t position, uint8_t buffer)
   {
      return write_n(5, address, M1PPOS, SetWORDval(position), buffer); // Writes 1 uint16_t and 1 uint8_t
   }

};

#endif
