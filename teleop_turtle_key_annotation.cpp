#include <ros/ros.h>			// ROS 기본 헤더 파일 include
#include <geometry_msgs/Twist.h>	// velocity message
#include <signal.h>			//시그널 헤더 파일 include
#include <stdio.h>			//입력/출력 파일 include
//if not defined, 중복정의를 피하기 위해 다음과 같이 처리함
#ifndef _WIN32				//OS가 WIN32이면 다음과 같이 include
# include <termios.h>			// 터미널 제어관련 헤더 파일
# include <unistd.h>			// 표준 심볼 상수 및 자료형 헤더 파일 
#else					//다른 OS의 경우 다음과 같이 include
# include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43		//각각의 keycode를 다음과 같이 정의한다.
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader				//키보드에서 받은 값을 읽는 클래스
{
public:
  KeyboardReader()
#ifndef _WIN32					//WIN32인 경우 다음 코드를 실행한다.
    : kfd(0)					//kfd = 0
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);				//터미널 속성을 얻는다
    struct termios raw;					//구조체 선언
    memcpy(&raw, &cooked, sizeof(struct termios));	//&cooked를 &raw로 구조체 크기만큼 복사
    raw.c_lflag &=~ (ICANON | ECHO);			//canonical 입력모드 사용, 입력을 다시 출력
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;	//라인 종료자(terminator) 로서 동작하는 제어문자
    raw.c_cc[VEOF] = 2;	//만일 EOF 문자가 라인의 처음에 존재된다면, 0바이트를 반환하여, 파일의 끝임을 지적
    tcsetattr(kfd, TCSANOW, &raw);			//터미널 속성을 설정
#endif
  }
  void readOne(char * c)			//키보드로 들어오는 문자 체크
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)					//키보드에서 코드로 정의하지 않은 값이 눌릴 경우
    {						//값에러가 발생했다는 것을 명시적으로 알려줌
      throw std::runtime_error("read failed");
    }
#else						//다른 OS의경우 아래 코드를 실행한다.
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);	//핸들을 입력받는다
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);	//에러가 없는 경우 0이 아닌 값을 리턴
      if(events > 0)					//에러가 없는 경우 다음을 실행
      {
        ReadConsoleInput(handle, &buffer, 1, &events);	//에러가 없는 경우 0이 아닌 값을 리턴
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)	//눌린 키의 주소값을 저장
        {
          *c = KEYCODE_LEFT;					//왼쪽화살표의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;					//위쪽화살표의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;					//오른쪽화살표의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;					//아래화살표의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
        {
          *c = KEYCODE_B;					//B의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
        {
          *c = KEYCODE_C;					//C의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
        {
          *c = KEYCODE_D;					//D의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
        {
          *c = KEYCODE_E;					//E의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
        {
          *c = KEYCODE_F;					//F의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
        {
          *c = KEYCODE_G;					//G의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;					//Q의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
        {
          *c = KEYCODE_R;					//R의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
        {
          *c = KEYCODE_T;					//T의 주소값 저장
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
        {
          *c = KEYCODE_V;					//V의 주소값 저장
          return;
        }
      }
    }
#endif
  }
  void shutdown()				// 종료 부분
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);		//kfd에 대한 터미널 속성 설정을 바로 변경, cooked는 속성을 저장할 주소다.
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;			//구조체 선언
#endif
};

KeyboardReader input;				//input에 클래스 할당

class TeleopTurtle
{
public:
  TeleopTurtle();				//속도 관련 변수 선언 및 창 크기 선언 함수
  void keyLoop();				//키보드 값 받는 함수

private:

  
  ros::NodeHandle nh_;				//nodehandle 선언
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;			//퍼블리셔 선언
  
};

TeleopTurtle::TeleopTurtle():			//변수값 설정
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);	//scale_angular에 값 할당
  nh_.param("scale_linear", l_scale_, l_scale_);	//scale_linear에 값 할당

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

void quit(int sig)				//종료 부분
{
  (void)sig;
  input.shutdown();				//KeyboardReader의 shutdown함수
  ros::shutdown();				//ros의 shutdown함수
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");	//문자열 출력
  TeleopTurtle teleop_turtle;			//teleop_turtle에 클래스 할당

  signal(SIGINT,quit);				//터미널 인터럽트 종료

  teleop_turtle.keyLoop();			//키보드 입력에 따라 터틀심을 움직임
  quit(0);					//프로그램을 끝낸다
  
  return(0);
}


void TeleopTurtle::keyLoop()			//키보드 입력을 받고 이에 따라 터틀심을 움직이는 함수
{
  char c;
  bool dirty=false;

  puts("Reading from keyboard");				//문자열 출력
  puts("---------------------------");
  puts("Use arrow keys to move the turtle. 'q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard
    try								//예외가 발생하게되면 catch로 이동
    {
      input.readOne(&c);					//키보드로 들어오는 문자 체크
    }
    catch (const std::runtime_error &)
    {
      perror("read():");					//read():read failed 메시지를 출력
      return;
    }

    linear_=angular_=0;						//변수값 초기화
    ROS_DEBUG("value: 0x%02X\n", c);				//입력된 키 출력
  
    switch(c)
    {
      case KEYCODE_LEFT:		//왼쪽 방향키를 누르면 각속도는 1
        ROS_DEBUG("LEFT");		//LEFT 메시지 출력
        angular_ = 1.0;			//각속도 = 1
        dirty = true;			//플래그 true
        break;
      case KEYCODE_RIGHT:		//오른쪽 방향키를 누르면 각속도는 -1
        ROS_DEBUG("RIGHT");		//RIGHT 메시지 출력
        angular_ = -1.0;		//각속도 = -1
        dirty = true;			//플래그 true
        break;
      case KEYCODE_UP:			//위쪽 방향키를 누르면 속도는 1
        ROS_DEBUG("UP");		//UP 메시지 출력
        linear_ = 1.0;			//속도 = 1
        dirty = true;			//플래그 true
        break;
      case KEYCODE_DOWN:		//아래쪽 방향키를 누르면 속도는 -1
        ROS_DEBUG("DOWN");		//DOWN 메시지 출력
        linear_ = -1.0;			//속도 = -1
        dirty = true;			//플래그 true
        break;
      case KEYCODE_Q:			//Q를 누르면 끝냄
        ROS_DEBUG("quit");		//quit 메시지 출력
        return;
    }
   

    geometry_msgs::Twist twist;			//twist에 속도 관련 함수 할당
    twist.angular.z = a_scale_*angular_;	//속도 변수 설정
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)				//터틀심을 움직이는 키를 눌렀을 때 메시지 출력
    {
      twist_pub_.publish(twist);    		//데이터 송신
      dirty=false;				//플래그 false
    }
  }


  return;
}



