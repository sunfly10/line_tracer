# line_tracer

### 1. 서론

본 프로젝트는 임베디드 환경에서 라인 스캔 카메라 데이터를 활용하여 트랙을 자율 주행하는 라인트레이서 시스템을 구현한 팀 프로젝트입니다. 

고정된 하드웨어 자원 내에서 실시간성을 확보하기 위해 Timer 인터럽트 기반의 DDA(Digital Differential Analyzer) 알고리즘을 직접 구현하여 스텝 모터를 정밀 제어한 것이 핵심입니다.

•개발 환경: C/C++ (Atmel Studio / AVR 기반 마이크로컨트롤러)

•주요 특징: DDA 기반 20kHz 모터 제어, 200Hz 제어 루프, 동적 임계값 기반 라인 인식 알고리즘

### 2. System Architecture & Control Logic

•DDA 알고리즘 기반 STEP 펄스 생성

Timer1 ISR(20kHz) 내에서 DDA 알고리즘을 수행하여 정밀한 스텝 펄스를 생성합니다.

메인 루프에서 계산된 좌/우 속도(b_left, b_right)를 기반으로 실시간 펄스 주기를 가변하여 선형적인 속도 제어를 구현했습니다.

•Robust 라인 인식 

(Dynamic Thresholding)환경 적응형 임계값: 조명 변화 및 빛 반사에 취약한 고정 임계값의 한계를 극복하기 위해, 매 프레임의 유효 구간 내 최댓값과 최솟값의 평균을 Threshold로 갱신하는 방식을 적용했습니다.

상태별 예외 처리: 

단일 차선 인식: 한쪽 차선을 놓칠 경우 조향이 급변하지 않도록 Center_ind를 단계적으로 보정하여 탈선을 방지합니다.

탈선 복구 모드: 대비차가 150 미만인 경우를 탈선으로 정의하고, 저속 후진을 통해 라인을 재탐색하는 복구 로직을 구현했습니다.

•가변 이득 및 주행 최적화 제어

에러 기반 가변 이득: 중심 오차(abs_err)가 8.0 미만인 직선 구간에서는 부드러운 조향(current_k = 15.0)을, 코너 구간에서는 강력한 조향(current_k = 80.0)을 주는 가변 이득 기법을 적용했습니다.

가감속 프로파일: 오차(err)와 오차 변화량(derr)을 동시에 분석하여 코너 진입 시점을 민감하게 포착하고 속도를 감속(-2000)하며, 직선 구간에서는 최대 속도(top_speed)까지 가속하여 랩 타임을 단축했습니다.

•정지선 감지 및 중복 인식 방지

화면 전체의 검은색 픽셀 개수(blackCount)를 분석하여 정지선을 판별합니다.

동일 정지선을 여러 번 인식하는 문제를 해결하기 위해 1,000 프레임 동안 재검출을 방지하는 Cooldown 카운트 로직을 도입했습니다.

### 3.Troubleshooting & Analysis

고속 주행 시 코너 이탈: err만으로 제어할 시 감속 시점이 늦어지는 문제를 발견했습니다. 

derr을 제어 조건에 추가하여 차량이 중심에서 벗어나는 속도를 민감하게 감지함으로써 고속 주행 안정성을 확보했습니다.

햇빛에 의한 광량 과다: 강한 외부 광선으로 인해 검은색 차선이 백색으로 오인되는 상황을 카메라 저항 조절을 통한 Bias 조절로 해결하며 하드웨어와 소프트웨어 간의 상호 보완적 해결 능력을 배양했습니다.

Trade-off 최적화: '랩 타임 단축을 위한 고속 주행'과 '안정적인 코너링' 사이의 상충 관계를 수많은 파라미터 튜닝을 통해 최적의 균형점을 도출했습니다.

### 4. Hardware Specifications

•Camera: TSL1401 Line Scan Camera (128 pixels) 

•Controller: AVR (ATmega series)

•Actuator: Step Motor (via DDA Control) 

•Control Frequency: 200Hz Loop / 20kHz Interrupt
Controller: AVR (ATmega series)Actuator: Step Motor (via DDA Control) Control Frequency: 200Hz Loop / 20kHz Interrupt 
