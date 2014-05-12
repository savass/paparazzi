#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define MODEM "/dev/ttyUSB0"
#define BAUDRATE B9600
#define BUFLEN 20
#define ENCODER_ZERO "800000"
#define SLEEP_T 20000  //Time to sleep (in nanosec)  after each command send to orion
int EncoderFullCircle1;
int EncoderSiderealRate1;

int EncoderFullCircle2;
int EncoderSiderealRate2;

void WriteToSerial(int fd, const char *str)
{
    int i = strlen(str);
    write(fd, str, i);
    if (i>0 && str[i-1]!='\r') write(fd, "\r", 1);
}

int init_serial_port() {

   struct termios tio;
    int fd;
    int flags;
    unsigned char c='D';

    memset(&tio,0,sizeof(tio));
    tio.c_iflag=0;
    tio.c_oflag=0;
    tio.c_cflag=CS8 | CREAD | CLOCAL;           // 8n1, see termios.h for more information
    tio.c_lflag=0;
    tio.c_cc[VMIN]=1;
    tio.c_cc[VTIME]=5;
    if((fd = open(MODEM , O_RDWR | O_NONBLOCK)) == -1){
        printf("Error while opening\n"); // Just if you want user interface error control
        return -1;
    }
    cfsetospeed(&tio,BAUDRATE);
    cfsetispeed(&tio,BAUDRATE);            // baudrate is declarated above
    tcsetattr(fd,TCSANOW,&tio);

 return fd;

}


int hex_to_int(char *hexstring) {
 int number = (int)strtol(hexstring, NULL, 16);
 return number;
}

void int_to_hex(int Inp, char *hexstring) {
sprintf(hexstring, "%X", Inp);
return;
}

void encode_axis_value(char *InStr, char *OutStr){

  //TODO check strlen of incoming str
 printf("in=%s\n",InStr);

//int i=strlen(InStr);
  //printf("inc str len=%d\n",i);
  //i=i-1;
  //int k=strlen(OutStr);
 OutStr[0]=InStr[4];
 OutStr[1]=InStr[5];
 OutStr[2]=InStr[2];
 OutStr[3]=InStr[3];
 OutStr[4]=InStr[0];
 OutStr[5]=InStr[1];
  /*int k=0;
  for (i;i>=0;i--){
   OutStr[k]=InStr[i];
   k++;
  }*/
}

void decode_axis_value (char InStr[BUFLEN], char OutStr[BUFLEN]) {
 int i=0;
 int k=0;
 char StrBuf[BUFLEN];
 //find the =***\r part
 for (i=0; i<BUFLEN; i++) {
  if (InStr[i] == '=') {
    i++;
    for ( i ;  i<BUFLEN  ; i++ ) {
      //printf("in looking %d -> %c\n",i,InStr[i]);
      if (InStr[i]=='\r') break;
      StrBuf[k]=InStr[i];
      k++;
    }
    if (InStr[i]=='\r') break;
    }
 }
 StrBuf[k]='\n';
 k=0;
 //now encode it

 OutStr[0]=StrBuf[4];
 OutStr[1]=StrBuf[5];
 OutStr[2]=StrBuf[2];
 OutStr[3]=StrBuf[3];
 OutStr[4]=StrBuf[0];
 OutStr[5]=StrBuf[1];
 //printf("out=%s\n",OutStr);
 /*
  * i=strlen(StrBuf);
 i=i-2; //for null char
  for ( i;  i>=0  ; i-- ) {
    OutStr[k]=StrBuf[i];
    k++;
  }
 OutStr[k]='\n';*/
}

void init_orion(int fd) {

    printf("Initiating Orion tracker..\n");
    //char c;
    char buffer[BUFLEN];
    char myOutbuffer[BUFLEN];
    //printf("Writing F1\n");
    WriteToSerial(fd,":F1\r");
    usleep(100000);
    read(fd, buffer,sizeof(buffer));
    memset(buffer,0,sizeof(buffer));

    //1st joint
    //read encoder full circle
    WriteToSerial(fd,":a1\r");
    usleep(100000);
    read(fd, buffer,sizeof(buffer));
    decode_axis_value(buffer,myOutbuffer);
    EncoderFullCircle1= hex_to_int(myOutbuffer);
    printf("Encoder counts per turn (1) in hex:\n");
    printf(myOutbuffer);
    printf("\n");
    printf("Encoder counts per turn (1) in int:%d\n",EncoderFullCircle1);
    memset(buffer,0,sizeof(buffer));
    memset(myOutbuffer,0,sizeof(myOutbuffer));

    //read idereal rate

    WriteToSerial(fd,":D1\r");
    usleep(100000);
    read(fd, buffer,sizeof(buffer));
    decode_axis_value(buffer,myOutbuffer);
    EncoderSiderealRate1= hex_to_int(myOutbuffer);
    printf("Sidereal rate (1) in hex:\n");
    printf(myOutbuffer);
    printf("\n");
    printf("Sidereal rate (1) in int:%d\n",EncoderSiderealRate1);
    memset(buffer,0,sizeof(buffer));
    memset(myOutbuffer,0,sizeof(myOutbuffer));

    //2nd joint
    WriteToSerial(fd,":F2\r");
    usleep(100000);
    read(fd, buffer,sizeof(buffer));
    memset(buffer,0,sizeof(buffer));
    WriteToSerial(fd,":a2\r");
    usleep(100000);
    read(fd, buffer,sizeof(buffer));
    decode_axis_value(buffer,myOutbuffer);
    EncoderFullCircle2= hex_to_int(myOutbuffer);

    printf("Encoder counts per turn (2) in hex:\n");
    printf(myOutbuffer);
    printf("\n");
    printf("Encoder counts per turn (2) in int:%d\n",EncoderFullCircle2);
    memset(buffer,0,sizeof(buffer));
    memset(myOutbuffer,0,sizeof(myOutbuffer));

    //printf("writing d1\n");
    WriteToSerial(fd,":D2\r");
    usleep(100000);
    read(fd, buffer,sizeof(buffer));
    decode_axis_value(buffer,myOutbuffer);
    EncoderSiderealRate2= hex_to_int(myOutbuffer);
    printf("Sidereal rate (2) in hex:\n");
    printf(myOutbuffer);
    printf("\n");
    printf("Sidereal rate (2) in int:%d\n",EncoderSiderealRate2);
    memset(buffer,0,sizeof(buffer));
    memset(myOutbuffer,0,sizeof(myOutbuffer));

}

void go_to(int JointNo, int JointAngle, int fd) {

  //Calculate joint integer
  int calc_angle;
  if (JointNo==1) {
    calc_angle= ((EncoderFullCircle1*JointAngle)/360) + hex_to_int(ENCODER_ZERO);
    printf("Pan go_to =%d\n",JointAngle);
  }else {
    calc_angle= ((EncoderFullCircle2*JointAngle)/360) + hex_to_int(ENCODER_ZERO);
    printf("Tilt go_to =%d\n",JointAngle);
  }
  printf("Angle int=%d\n",calc_angle);

  //convert to hex
  char ang_hex_buf[20];
  int_to_hex(calc_angle,ang_hex_buf);
  //printf("Angle hex=%s\n",ang_hex_buf);
  //encode hex_to_int
  char ang_hex_encoded[20];
  memset(ang_hex_encoded,0,sizeof(ang_hex_encoded));
  encode_axis_value(ang_hex_buf,ang_hex_encoded);
  //printf("Angle hex (enc)=%s\n",ang_hex_encoded);
  //create send string
  char strbuf[BUFLEN];
  char buffer[BUFLEN];
  //drive to positin
  //L<axis>
  //return 0;
  sprintf(strbuf, ":L%d\r", JointNo);
//>>>>>WriteToSerial(fd,strbuf);
  //printf("\n");
  //printf("written:%s",strbuf);
  //printf("\n");
   memset(strbuf,0,sizeof(strbuf));
  usleep(SLEEP_T);
  read(fd, buffer,sizeof(buffer));
  //printf("\n");
  //printf("Got:\n");
  //printf("%s",buffer);
  //printf("\n");
  //G<axis>00
  sprintf(strbuf, ":G%d00\r", JointNo);
  WriteToSerial(fd,strbuf);
  //printf("\n");
  //printf("written=%s",strbuf);
  printf("\n");
   memset(strbuf,0,sizeof(strbuf));
  usleep(SLEEP_T);
  //S<axis><pos>
  sprintf(strbuf, ":S%d%s\r", JointNo,ang_hex_encoded);
  WriteToSerial(fd,strbuf);
  printf("\n");
  printf("written=%s",strbuf);
  printf("\n");
  memset(strbuf,0,sizeof(strbuf));
  usleep(SLEEP_T);
  //WriteToSerial(fd,":S1\r");
  //J<axis>
  sprintf(strbuf, ":J%d\r", JointNo);
  WriteToSerial(fd,strbuf);
  printf("\n");
  printf("written=%s",strbuf);
  printf("\n");
  memset(strbuf,0,sizeof(strbuf));
  usleep(SLEEP_T);
  //write to serial port

}