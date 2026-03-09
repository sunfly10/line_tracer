//https://youtu.be/7Vwpv0AkzIM?si=zFL_LHnOIKnRoF25

#include <stdint.h> //uint, int형 비트 명시를 위해 사용
//속도 관련 설정
#define base_speed 5000 //차체의 기본 속도    ;5000 = 14.3s ;  4500 = 15.69   ; 4000 =  17.5  ; 3500  =  19.5 ;
#define top_speed  9000 //차체가 가속할 시 도달할 수 있는 최대 속도
#define accel_rate 140 //샘플링 시간(0.005초)마다의 가속값

#define m 1 //duration
#define a 20000 //dda 알고리즘(11주차 강의자료)에서 a 부분

//정지선 인식 + 랩 카운트 설정
#define stopline_blackcount_th 95 //정지선으로 인식할 때 최소 95개를 검정색으로 인식해야지 정지선으로 판단
#define stopline_no_detect 1000 //선을 계속 인식하게 하지 말고 한번 봤으면 특정 시간동안 보지 못하게 하기 위해 사용
#define stopline_hits 2 //1랩(1) + 2랩(2) --> 2면 정지

unsigned int camera_data[140] = {0}; //라인 센서 데이터 값을 받기 위한 배열

//sampling 관련 설정
uint32_t MicroSampleTime;
uint32_t start_time;
float sample_time = 0.005f; //200hz, 더 작은 sampling time은 loop문이 도는 시간보다 짧아서 코드가 실행되는 것에 어려움이 있다.

// 필요없다고 판단, 내일 주행후 다시 판단할 것 int raw_center_ind = 63; 
int center_ind = 63; //중심 인덱스 처음에는 63으로
float current_speed; //현재 속도 관련 파라미터

//dda 알고리즘(11주차 강의자료)에서 필요로 하는 변수들
volatile unsigned int cnt1_left = 0;
volatile unsigned int cnt2_left = 0;
volatile unsigned int cnt1_right = 0;
volatile unsigned int cnt2_right = 0;
volatile unsigned int b_left = 0; //왼쪽 바퀴의 dda 알고리즘에서의 b
volatile unsigned int b_right = 0; //오른쪽 바퀴의 dda 알고리즘에서의 b

bool no_2line = false; //탈선 시 true, 처음에는 탈선이 아니라고 가정(false)

//정지선 감지 상태 설정
static bool stopline_detected = false; //stopline 감지됐는지 알기 위해
static int  stopline_hits_number = 0; //총 3번을 hit해야 정지
static int  stopline_no_detect_count = 1000; //정지선을 몇 프레임동안 무시할 지(중복 검출 방지)
static bool stop_state = false; //정지상태인지 알기 위해

void ADCinit();
void ADC_Get(unsigned char num);
void search_center();
void control_motor();
void timer1_init();

void timer1_init() { 
  //CTC mode
  TCCR1B &= ~_BV(WGM13);
  TCCR1B |= _BV(WGM12);
  TCCR1A &= ~_BV(WGM11);
  TCCR1A &= ~_BV(WGM10);
  //N=8
  TCCR1B &= ~_BV(CS12);
  TCCR1B |= _BV(CS11);
  TCCR1B &= ~_BV(CS10);

  OCR1A = 99; //a=20kHz로 설정했으므로, f = 16MHz / (N * (1 + OCR1A)  
  TCNT1 = 0; //맨 처음에 tcnt1값 초기화
  //인터럽트 flag와 enable 활성화
  TIMSK1 |= _BV(OCIE1A);
  TIFR1 |= _BV(OCF1A);
}
//인터럽트에서 dda 알고리즘 수행
ISR(TIMER1_COMPA_vect) {
  cnt1_left += 1;
  if (cnt1_left > m) PORTD &= ~_BV(PD0);
  cnt2_left += b_left;
  if (cnt2_left >= a) {
    PORTD |= _BV(PD0);
    cnt2_left -= a;
    cnt1_left = 0;
  }
  cnt1_right += 1;
  if (cnt1_right > m) PORTD &= ~_BV(PD2);
  cnt2_right += b_right;
  if (cnt2_right >= a) {
    PORTD |= _BV(PD2);
    cnt2_right -= a;
    cnt1_right = 0;
  }
}

void ADCinit() {
  ADMUX = 0;
  //AVCC를 AREF로 사용
  ADMUX &= ~_BV(REFS1);
  ADMUX |= _BV(REFS0);

  ADMUX &= ~_BV(ADLAR); //오른쪽 정렬 방식 사용
  ADCSRA |= _BV(ADEN); //ADC enable

  ADCSRA &= ~_BV(ADSC); //start conversion 0
  ADCSRA &= ~_BV(ADATE); //auto trigger enable = 0 --> single conversion
  //adc interrupt disable
  ADCSRA |= _BV(ADIF);
  ADCSRA &= ~_BV(ADIE);
  //divison factor 32 --> 500kgz clock
  ADCSRA |= _BV(ADPS2);
  ADCSRA &= ~_BV(ADPS1);
  ADCSRA |= _BV(ADPS0);
}

void ADC_Get(unsigned char num) {
  ADCSRA |= (1 << ADSC); //start conversion
  //변환 종료까지 기다린 후 데이터를 특정 index에 저장
  while (ADCSRA & (1 << ADSC));
  camera_data[num] = ADCW;
}

void search_center() {
  int darkest_val = 1023; //1023부터 시작해야지 점점 작은 값을 찾을 수 있다
  int brightest_val = 0; //0부터 시작해야지 점점 큰 값을 찾을 수 있다

  for (int i = 5; i < 123; i++) { //0~4, 124~127은 무시(양끝값은 크게 중요하지 않다고 생각, 선을 구분하기 위해서는 가운데에서 근방을 사용하는 것이 효율적이다라고 생각합니다)
    int v = camera_data[i]; //저장한 데이터에서 값을 받아온다
    if (v < darkest_val) darkest_val = v; //이 숫자보다 작으면 최소값 --> 최종적으로 for문을 완료 시 5~123번 index에서 가장 작은 값을 얻을 수 있다
    if (v > brightest_val) brightest_val = v; //이 숫자보다 크면 최대값 --> 최종적으로 for문을 완료 시 5~123번 index에서 가장 큰 값을 얻을 수 있다
  }
  //탈선 감지, max 값과 min 값의 차이가 150보다 작으면 탈선으로 간주(흰선만 보는 상태)
  if ((brightest_val - darkest_val) < 150) {
    //느린 속도로 후진
    PORTD &= ~_BV(PD1);
    PORTD |= _BV(PD3);
    b_left = 500;
    b_right = 500;
    current_speed = base_speed;

    no_2line = true; //2line을 모두 감지한 상태가 아니다
    stopline_detected = false; //당연히 stopline을 감지한 상태도 아니다
    return;
  }
  no_2line = false; //탈선이 아니라면 2line 모두 감지한 상태이다
  int threshold = (brightest_val + darkest_val) / 2; //현재 상태에서 최솟값, 최대값의 절반을 기준으로 흑 백을 나눌 예정
  //threshold를 기준으로 작은 것의 개수를 세서 정지선 탐지에 사용
  int blackCount = 0;
  for (int i = 5; i < 123; i++) {
    if (camera_data[i] < threshold) blackCount++;
  }
  stopline_detected = (blackCount >= stopline_blackcount_th); //우리가 정한 검정색 개수에 대한 threshold보다 크거나 같으면 stopline을 감지했다고 판단

  //차선 중심 추정, -1이 유지된다면 한쪽선을 감지하지 못한 상태이다(혹시 모를 대비해서 한번 더 판단을 해준다, 이중 판단)
  int left_ind = -1;
  int right_ind = -1;

  for (int i = 5; i < 64; i++) { //5~63이 왼쪽 영역이므로, 왼쪽 영역에서 가운데에 가장 가까운 인덱스를 찾기 위해 +1 사용
    if (camera_data[i] < threshold && camera_data[i + 1] < threshold) left_ind = i;
  }
  for (int i = 122; i > 63; i--) { //64~122이 오른쪽 영역이므로, 오른쪽 영역에서 가운데에 가장 가까운 인덱스를 찾기 위해 -1 사용
    if (camera_data[i] < threshold && camera_data[i - 1] < threshold) right_ind = i;
  }
  if (left_ind == -1 && right_ind != -1) { //왼쪽 선을 인식 못했을 경우, center_ind를 0으로 해서 왼쪽 회전하게 한다
    center_ind -= 1;
    if (center_ind < 0) center_ind = 0;
  }
  else if (left_ind != -1 && right_ind == -1) { //오른쪽 선을 인식 못했을 경우, center_ind를 127으로 해서 오른쪽 회전하게 한다
    center_ind += 1;
    if (center_ind > 127) center_ind = 127;
  }
  else if (left_ind != -1 && right_ind != -1) { //두번의 판단을 통해 두 선을 모두 인식했다는 것을 확실시하면 오른쪽 왼쪽 두 인덱스의 절반을 현재 인덱스로 설정한다.
    center_ind = (right_ind + left_ind) / 2;
  }
}

void control_motor() {
  //이미 정지 상태면 그대로 유지
  if (stop_state) {
    b_left = 0;
    b_right = 0;
    return;
  }

  if (stopline_no_detect_count > 0) stopline_no_detect_count--; //감지 한번 했다면 카운트를 감소시켜 0이 될때까지는 감지 하지 않게 한다
  if (stopline_detected && stopline_no_detect_count == 0) { //감지했고, 이전에 감지한 적이 없다면
    stopline_hits_number++; //stopline을 감지했고 + --> stopline count를 +1한다
    stopline_no_detect_count = stopline_no_detect; //count에 위에서 정한 값을 넣어서 정해진 숫자까지 감지 못하게 한다.

    if (stopline_hits_number >= stopline_hits) { 
      stop_state = true; //3번 이상 hit했다면 무조건 멈춰야한다. 즉, 정지상태
      b_left = 0; //왼쪽 오른쪽 b값 0
      b_right = 0;
      return;
    }
  }
  //계수 값들은 여러 실험을 통해 얻은 최적의 값
  static float prev_err = 0.0f; //static --> 처음에 0.0 float형으로 초기화
  float err = (float)center_ind - 63.5f; //error를 중심값(63.5)와의 차이로 얻는다.
  float abs_err = (err < 0) ? -err : err; //error 값이 음수면 -부호 붙여서 양수로
  float derr = err - prev_err; //D controller --> 에러 변화량을 사용하므로
  prev_err = err; //이전 에러를 현재에러로 업데이트
  float abs_derr = (derr < 0) ? -derr : derr; //에러 변화량이 음수면 양수로 변환
  float current_k = (abs_err < 8.0f) ? 15.0f : 80.0f; //에러변화량이 8.0을 기준으로 kd를 업데이트(직선구간에서는 많이 움직이지 않게, 커브 구간에서는 많이 움직여서 회전하면서 이동하게끔)
  const float straight_e  = 2.5f; //에러 2.5까지로 직선구간으로 정의
  const float straight_de = 1.2f; //에러 변화량 1.2까지로 직선구간으로 정의
  bool is_straight = (abs_err < straight_e) && (abs_derr < straight_de); //두 조건에 부합하면 직선으로 인식을 한다
  const float corner_e  = 2.0f; //에러 3.0부터 커브구간으로 정의
  const float corner_de = 0.05f; //차량이 얼마나 빠르게 중심에서 벗어나고 있는지, 회전이 시작되는 순간을 더 세밀하게 인식
  bool corner_enter = (abs_err > corner_e) || (abs_derr > corner_de); ////두 조건에 부합하면 회전구간으로 인식을 한다
  if (corner_enter){ //코너라면
    current_speed -= 2000.0f; //회전시에 직진 속도를 그대로 수행할 경우 트랙을 벗어날 위험이 있다, 따라서 감소시켜서 진행
    if (current_speed < base_speed) current_speed = base_speed; //기본속도보다 낮지 않게 한다.
  }
  else if (is_straight){ //직선 구간이라면
    current_speed += (float)accel_rate; //가속도로 정한 값 만큼 증가시킨다
    if (current_speed > top_speed) current_speed = top_speed; //최고 속도를 넘지 못하게 한다.
  }
  else { //두 경우 모두 아닌 애매한 경우라면
    current_speed -= 25.0f; //정말 약간의 속도를 줄여 직선, 커브 구간을 인식하게 한다. 
    if (current_speed < base_speed) current_speed = base_speed; //기본속도보다 낮지 않게 한다.
  }
  int v = (int)current_speed; //목표 속도 저장
  //회전 시 안쪽 바깥쪽 최저 속도를 다음과 같이 제한
  const int min_in_wheel = 900;
  const int min_out_wheel = 1500;

  //전진하게 방향을 설정
  PORTD |= _BV(PD1);
  PORTD &= ~_BV(PD3);

  //63, 64번이 중심 인덱스인 경우만 직선 주행
  if (center_ind >= 63 && center_ind <= 64) {
    b_left = v;
    b_right = v;
    return;
  }

  //중심 인덱스가 65이상부터는 오른쪽 회전 --> 왼쪽 바퀴 속도가 더 빨라야 한다!
  if (center_ind > 64){
    int out = v;
    int in  = v - (int)(current_k * (center_ind - 63.5f));
    //최저 속도보다 낮지 않게 설정
    if (out < min_out_wheel) out = min_out_wheel;
    if (in < min_in_wheel) in = min_in_wheel;
    //왼쪽이 바깥쪽
    b_left = out;
    b_right = in;
  } 
  else{ //중심 인덱스가 62이하부터는 왼쪽 회전 --> 오른쪽 바퀴 속도가 더 빨라야 한다 + 오른쪽 회전 상황에서는 몇초이내에 바로 왼쪽 회전을 해야하므로 빠르게 회전하는 것이 중요하다고 생각 --> +500/ -800의 이유
    int out = v;
    int in  = v - (int)(current_k * (63.5f - center_ind));
    //최저 속도보다 낮지 않게 설정
    if (out < min_out_wheel+500) out = min_out_wheel+500; 
    if (in < min_in_wheel-800) in = min_in_wheel-800;
    //오른쪽이 바깥쪽
    b_left = in;
    b_right = out;
  }
}

void setup() {
  //Serial.begin(115200); //혹시 특정 파라미터 값을 알고 싶다면 사용한다.
  ADCinit();
  //13주차 프로젝트 강의자료 기준으로 in/output 설정
  DDRA |= (1 << PA0) | (1 << PA1);
  DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3);
  DDRH |= (1 << PH0);
  PORTH &= ~_BV(PH0); // /EN: active low
  timer1_init();
  sei(); //global interrupt enable
  //sampling time을 uint32 type으로 변경해준다
  MicroSampleTime = (uint32_t)(sample_time * 1e6);
  start_time = micros() + MicroSampleTime;

  current_speed = base_speed; //맨 처음에는 기본 속도로 현재 속도를 설정
}

void loop() {
  //line scan camera의 순서: si high --> clk high --> si low 
  PORTA |= (1 << PA1);
  PORTA |= (1 << PA0);
  PORTA &= ~(1 << PA1);

  delayMicroseconds(2); //처음 시작 시 약간의 delay가 필요하다!!
  for (int i = 0; i < 129; i++) { //128개의 sensor 값을 받아온다
    ADC_Get(i);
    //clk low --> clk high 를 데이터를 다 가져올 때까지 반복, clk하나당 하나의 data이다
    PORTA &= ~(1 << PA0); 
    PORTA |=  (1 << PA0);
  }
  PORTA &= ~(1 << PA0); //data를 다 받았다면 clk를 low로 하여 끝낸다
  search_center(); //중심 인덱스를 찾는다
  if (no_2line == false) control_motor(); //중심 인덱스를 찾았다면 2line을 모두 찾았으니 모터 제어를 실시한다.
  //uint32 type의 경우 start_time을 현재 시각이 넘으면 오버플로우가 발생해 앞자리가 1이 된다 --> 0x8000000과 bitwise and를 할 시 1이 출력 --> ! --> 0 --> while이 끝난다.
  while (!((start_time - micros()) & 0x80000000));
  start_time += MicroSampleTime;
}
