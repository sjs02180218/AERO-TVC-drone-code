// Teensy 4.1 | Coaxial TVC | TVC_Hover_Plant_Coaxial_3D(5).slx
// 보드: Teensy 4.1. 라이브러리: Adafruit BNO055, Adafruit Unified Sensor.
// 첫 PWM 확인은 프로펠러를 뗀 상태에서 한다.
// CRM2413 1300KV / 3S / 9050. 기본 ESC 변환은 공개 합산 추력으로 만든 추정 곡선.
// 실측표로 전환하려면 fan1Curve/fan2Curve를 채우고 usePublished3sEstimate를 false로 둔다.
// 프로펠러 없는 PWM 확인: 기준 자세 설정 후 ARM OFF/스로틀 최저에서
// 시리얼 모니터(줄바꿈)로 BENCH A 또는 BENCH B를 보낸 뒤 아밍한다.
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace config {
constexpr bool useSbus = true;
constexpr int throttleChannel = 2, armChannel = 5;  // SBUS CH3, CH6, Serial1 RX1(D0)
constexpr int pinThrottle = 2, pinArm = 3;         // 별도 PWM 모드
constexpr int pinEscA = 5, pinEscB = 4;
constexpr int pinJ1 = 6, pinJ2 = 7;
constexpr int sbusMin = 172, sbusMax = 1811;
constexpr int throttleMinUs = 1134, throttleLowUs = throttleMinUs + 10;
constexpr int armOnUs = 1400, armOffUs = 1600;      // 낮은 쪽이 ARM ON
constexpr int escStopUs = 1000, escMaxUs = 2000;
constexpr int benchMaxUs = 1200, benchRisePerTickUs = 4;
constexpr int j1CenterUs = 1516, j2CenterUs = 1510;

constexpr float j1UsPerDeg = 10.0f, j2UsPerDeg = 10.0f;
constexpr int j1PwmSign = +1, j2PwmSign = -1;       // PWM 증가가 그림의 +각과 반대이면 -1.
static_assert((j1PwmSign==1 || j1PwmSign==-1) && (j2PwmSign==1 || j2PwmSign==-1),
              "Servo signs must independently be +1 or -1.");
constexpr bool servoGeometryConfirmed = true;      // 중립, 부호, ±20° 간섭을 확인하는 설정.
constexpr bool escAIsModelFan1 = true;              // 모델 fan1 반작용 -X, fan2 +X.
constexpr bool fanOrderConfirmed = true;           // 위 D4/D5 매핑을 사용.
constexpr float previewLimitDeg = 20.0f;           // DISARM 및 지상시험 서보 명령 범위: ±20°.
constexpr float tiltCutoffDeg = 35.0f;
constexpr uint32_t rcTimeoutUs = 100000, escInitUs = 2000000;
constexpr uint32_t periodUs = 10000, maxGapUs = 30000;
constexpr uint8_t bnoAddress = 0x28;                // Wire: SDA D18, SCL D19
constexpr float rad = 0.0174532925199433f;
constexpr unsigned referenceSamples = 50;
constexpr float referenceMinAccel = 9.0f, referenceMaxAccel = 10.7f;
constexpr float referenceMinUpCos = .98f, referenceMaxGyro = .055f;

enum class ReferenceProp { Gws8040, Gws9050 };
constexpr ReferenceProp referenceProp = ReferenceProp::Gws9050;
constexpr float referenceLoadedVolts = 11.1f;       // 추정 곡선의 고정 기준 전압. 전압 계측 없음.
constexpr bool usePublished3sEstimate = true;

struct ThrustPoint { float newtons; int microseconds; };
// 3S에서 각 팬의 측정값을 추력과 펄스가 모두 증가하는 순서로 기입한다.
// 0 N은 반드시 첫 줄의 정지 펄스. 실측 모드에서는 양의 추력점도 필요하다.
constexpr ThrustPoint fan1Curve[] = {{0.0f, escStopUs}};
constexpr ThrustPoint fan2Curve[] = {{0.0f, escStopUs}};
}

// SLX의 Model Workspace와 MATLAB Function1/2에서 옮긴 상수.
namespace model {
constexpr float dt = 0.01f;
constexpr float g = 9.80665f, mass = 0.8f;
constexpr float tPeak = 8.0f, tFan1Max = 8.0f, tFan2Max = 8.0f;
constexpr float kq1 = 0.012f, kq2 = 0.012f;
constexpr float maxServoDeg = 20.0f, servoRateDegS = 400.0f;
constexpr float fanRiseNs = 30.0f, fanFallNs = 40.0f;

constexpr float kpRoll = 1.5f, kdRoll = 0.81f;
constexpr float kpAtt = 30.0f, kdAtt = 0.0f; // 24.1
constexpr float kvNorth = 0.0f, kvEast = 0.0f;
constexpr float kaNorth = 0.0f, kaEast = 0.0f, tauAcc = 0.060f;
constexpr float targetTiltDeg = 8.0f;
}

namespace reference3s {
struct Point { float volts, thrustGf; };
// CRM2413A performance data, GWS dual. 합산 정지 추력, ESC 펄스 폭 정보 없음.
// https://manuals.plus/asin/B07Q6JPJQW
// https://toyscentral.ie/products/crm2413a-brushless-metal-motor-contrarotating-motor-crm2413-1300kv-for-airplane-models-wprop-adapter-two-propellers-work-with-two-esc-bring-strong-power-supplywlarge-thrust-for-rc-flying-models-97607164
constexpr Point gws8040[] = {{10.0f,1082.0f},{12.0f,1188.0f},{12.6f,1385.0f}};
constexpr Point gws9050[] = {{11.1f,1420.0f},{12.0f,1582.0f},{12.6f,1696.0f}};

template <size_t N>
float totalAt(const Point (&points)[N], float volts) {
  if (!isfinite(volts) || volts<points[0].volts || volts>points[N-1].volts) return 0;
  for (size_t i=1;i<N;++i) {
    if (volts<=points[i].volts) {
      const Point &a=points[i-1], &b=points[i];
      const float fraction=(volts-a.volts)/(b.volts-a.volts);
      return (a.thrustGf+fraction*(b.thrustGf-a.thrustGf))*0.001f*model::g;
    }
  }
  return 0;
}

// 각 팬이 합산 최대추력의 절반, 회전수는 정규화한 PWM 명령에 비례한다고 가정.
// T_fan = (T_pair / 2) * ((PWM - 정지 펄스) / 펄스 범위)^2의 역.
int estimatedPulse(float requestedN,float pairReferenceN) {
  if (!(pairReferenceN>0) || !isfinite(requestedN)) return 0;
  const float fraction=fminf(1.0f,fmaxf(0.0f,2.0f*requestedN/pairReferenceN));
  return config::escStopUs+lroundf((config::escMaxUs-config::escStopUs)*sqrtf(fraction));
}
}

struct Vec { float x, y, z; };
struct Quat { float w, x, y, z; };
struct ImuSample { Vec accelB, gyroB; Quat qWB; };
struct Command { float fan1, fan2, j1, j2; };
struct RcInput { bool valid; int throttleUs, armUs; };
struct UprightCheck { float accelNorm, gyroNorm, upCos; };
struct PulseCapture { volatile uint32_t rise = 0, stamp = 0; volatile uint16_t us = 0; };
enum class ArmState { Disarmed, Initializing, Idle, Active, Bench };
enum class BenchEsc { None, A, B };

class Tvc {
public:
  void setup();
  void loop();
private:
  static constexpr float halfRoot2 = 0.7071067811865475f;
  static constexpr Quat uprightQ = {halfRoot2, 0, halfRoot2, 0};
  static constexpr Vec inertia = {0.003f, 0.016f, 0.016f};
  static constexpr Vec pivot = {-0.12f, 0, 0};
  static constexpr Vec link1 = {-0.084f, 0, 0}, link2 = {-0.157f, 0, 0};
  static constexpr int grid = 512;
  static PulseCapture throttlePulse, armPulse;
  static void throttleIsr() { capture(throttlePulse, config::pinThrottle); }
  static void armIsr() { capture(armPulse, config::pinArm); }
  static void capture(PulseCapture &pulse, int pin);
  static bool readRegisters(uint8_t reg, uint8_t *data, uint8_t size);
  static bool readImu(ImuSample &sample);
  static int16_t signed16(const uint8_t *p) { return int16_t(uint16_t(p[0]) | (uint16_t(p[1]) << 8)); }
  static float clamp(float v, float lo, float hi) { return fminf(hi, fmaxf(lo, v)); }
  static Vec plus(Vec a, Vec b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
  static Vec minus(Vec a, Vec b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
  static Vec scale(Vec v, float s) { return {v.x * s, v.y * s, v.z * s}; }
  static float dot(Vec a, Vec b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
  static float norm(Vec v) { return sqrtf(dot(v, v)); }
  static Vec cross(Vec a, Vec b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
  }
  static Vec sensorToBody(Vec v) { return {-v.x, v.z, v.y}; }
  static Quat multiply(Quat a, Quat b);
  static Quat conj(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }
  static Vec rotate(Quat q, Vec v) {
    const Vec u{q.x, q.y, q.z};
    return plus(v, scale(cross(u, plus(cross(u, v), scale(v, q.w))), 2.0f));
  }
  static Quat matrixToQuat(Vec n, Vec e, Vec d);
  static Vec applyJ(Vec v) { return {v.x*inertia.x,v.y*inertia.y,v.z*inertia.z}; }
  static Vec applyJinv(Vec v) { return {v.x/inertia.x,v.y/inertia.y,v.z/inertia.z}; }
  static const char *stateName(ArmState state);
  template <size_t N> static bool validCurve(const config::ThrustPoint (&curve)[N]);
  template <size_t N> static int thrustPulse(const config::ThrustPoint (&curve)[N], float n);
  void serviceSbus();
  void serviceBenchSerial(bool canSelect);
  RcInput readRc() const;
  bool acquireReference(const ImuSample &s, const char *&waitReason);
  void resetReferenceSamples();
  void logWaitUpright(RcInput rc, ImuSample s, UprightCheck check, const char *reason);
  Command allocate(Vec alpha, Quat q, Vec omega, float totalN, float cap1, float cap2) const;
  Vec updateController(const ImuSample &s, Quat q, bool integrate);
  void startEsc(uint32_t now);
  void disarm();
  void writeServo(float a, float b, float maximumDeg);
  void log(const char *status, RcInput rc, float tiltDeg, float totalN, Command u);
  static float slew(float current, float target, float rising, float falling) {
    return current + clamp(target-current,-falling*model::dt,rising*model::dt);
  }

  Adafruit_BNO055 bno{55, config::bnoAddress, &Wire};
  Servo escA, escB, servo1, servo2;
  uint8_t sbusFrame[25]{}, sbusIndex = 0;
  uint32_t sbusByteAt = 0, sbusAt = 0, nextTick = 0, lastTick = 0, escAt = 0, lastLog = 0;
  int sbusThrottle = 0, sbusArm = 0;
  bool sbusValid = false, imuStarted = false, motorReady = false, offSeen = false;
  bool curve1Valid = false, curve2Valid = false;
  bool referenceReady = false, previousSample = false;
  unsigned referenceCount = 0;
  ArmState state = ArmState::Disarmed;
  BenchEsc benchEsc = BenchEsc::None;
  char serialLine[16]{};
  uint8_t serialLength = 0;
  bool serialOverflow = false;
  int benchPulseUs = config::escStopUs;
  Vec upSum{}, eastSum{}, velocity{}, accFiltered{}, gyroPrev{};
  Quat qNW{}, qPrev{};
  float j1Applied = 0, j2Applied = 0, fan1Applied = 0, fan2Applied = 0;
  float fan1Cap = 0, fan2Cap = 0;
  float referencePairN = 0;
  int escAPulse = 0, escBPulse = 0;
  float cosine[2*grid+1], sine[2*grid+1];
};

constexpr Quat Tvc::uprightQ;
constexpr Vec Tvc::inertia, Tvc::pivot, Tvc::link1, Tvc::link2;
PulseCapture Tvc::throttlePulse, Tvc::armPulse;

void Tvc::capture(PulseCapture &pulse, int pin) {
  const uint32_t now = micros();
  if (digitalRead(pin)) {
    pulse.rise = now;
  } else if (pulse.rise) {
    const uint32_t width = now - pulse.rise;
    pulse.rise = 0;
    if (width >= 800 && width <= 2200) {
      pulse.us = width;
      pulse.stamp = now;
    }
  }
}

void Tvc::serviceSbus() {
  if (!config::useSbus) return;
  while (Serial1.available()) {
    const uint32_t now = micros();
    if (now - sbusByteAt > 3500) sbusIndex = 0;
    sbusByteAt = now;
    const uint8_t byte = Serial1.read();
    if (sbusIndex == 0 && byte != 0x0f) continue;
    sbusFrame[sbusIndex++] = byte;
    if (sbusIndex < sizeof(sbusFrame)) continue;
    sbusIndex = 0;
    if (sbusFrame[24] != 0 && (sbusFrame[24] & 0x0f) != 4) {
      sbusValid = false;
      continue;
    }
    const auto channelUs = [this](int channel) {
      const int bit = 11 * channel, k = 1 + bit/8;
      const uint32_t bytes = uint32_t(sbusFrame[k]) | (uint32_t(sbusFrame[k+1]) << 8)
                            | (uint32_t(sbusFrame[k+2]) << 16);
      const long raw = (bytes >> (bit % 8)) & 0x7ff;
      return int(clamp(map(raw, config::sbusMin, config::sbusMax, 1000, 2000), 1000, 2000));
    };
    sbusValid = (sbusFrame[23] & 0x0c) == 0;
    sbusThrottle = channelUs(config::throttleChannel);
    sbusArm = channelUs(config::armChannel);
    sbusAt = now;
  }
}

RcInput Tvc::readRc() const {
  if (config::useSbus) return {sbusValid && micros()-sbusAt < config::rcTimeoutUs, sbusThrottle, sbusArm};
  noInterrupts();
  const uint32_t t = throttlePulse.stamp, a = armPulse.stamp;
  const int throttle = throttlePulse.us, arm = armPulse.us;
  interrupts();
  const uint32_t now = micros();
  return {t && a && now-t < config::rcTimeoutUs && now-a < config::rcTimeoutUs, throttle, arm};
}

void Tvc::serviceBenchSerial(bool canSelect) {
  // 명령은 USB에서 한 줄로 읽는다. 아밍 중 받은 줄은 버린다.
  for (int i=0; i<32 && Serial.available(); ++i) {
    const int byte=Serial.read();
    if (byte=='\r' || byte=='\n') {
      if (canSelect && !serialOverflow) {
        serialLine[serialLength]='\0';
        if (strcmp(serialLine,"BENCH A")==0) benchEsc=BenchEsc::A;
        if (strcmp(serialLine,"BENCH B")==0) benchEsc=BenchEsc::B;
        if (strcmp(serialLine,"BENCH OFF")==0) benchEsc=BenchEsc::None;
      }
      serialLength=0;
      serialOverflow=false;
    } else if (!canSelect || serialLength>=sizeof(serialLine)-1) {
      serialOverflow=true;
    } else if (!serialOverflow) {
      serialLine[serialLength++]=char(byte);
    }
  }
}

bool Tvc::readRegisters(uint8_t reg, uint8_t *data, uint8_t size) {
  Wire.beginTransmission(config::bnoAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(config::bnoAddress, size) != size) return false;
  for (uint8_t i=0; i<size; ++i) data[i] = Wire.read();
  return true;
}

bool Tvc::readImu(ImuSample &sample) {
  uint8_t block[20];
  // 페이지 0: 가속도와 자이로~쿼터니언을 각각 읽고 통신 실패를 검출한다.
  uint8_t accel[6];
  if (!readRegisters(0x08, accel, sizeof(accel)) || !readRegisters(0x14, block, 20)) return false;
  constexpr float gyroScale = config::rad / 16.0f;
  sample.accelB = sensorToBody({.01f*signed16(accel),.01f*signed16(accel+2),.01f*signed16(accel+4)});
  sample.gyroB = sensorToBody({gyroScale*signed16(block),gyroScale*signed16(block+2),gyroScale*signed16(block+4)});
  const Quat qWS{
      signed16(block+12)/16384.0f,signed16(block+14)/16384.0f,
      signed16(block+16)/16384.0f,signed16(block+18)/16384.0f};
  const float n = sqrtf(qWS.w*qWS.w+qWS.x*qWS.x+qWS.y*qWS.y+qWS.z*qWS.z);
  if (!(n >= .8f && n <= 1.2f)) return false;
  // q_WB = q_WS * q_SB. 센서->기체 행렬은 180° 회전이어서 역도 같은 회전이다.
  sample.qWB = multiply({qWS.w/n,qWS.x/n,qWS.y/n,qWS.z/n}, {0,0,halfRoot2,halfRoot2});
  return true;
}

Quat Tvc::multiply(Quat a, Quat b) {
  const Vec av{a.x,a.y,a.z}, bv{b.x,b.y,b.z};
  const Vec v = plus(plus(scale(bv,a.w),scale(av,b.w)),cross(av,bv));
  return {a.w*b.w-dot(av,bv),v.x,v.y,v.z};
}

Quat Tvc::matrixToQuat(Vec n, Vec e, Vec d) {
  const float trace = n.x+e.y+d.z;
  Quat q{};
  if (trace > 0) {
    const float s = 2*sqrtf(1+trace);
    q = {s*.25f,(d.y-e.z)/s,(n.z-d.x)/s,(e.x-n.y)/s};
  } else if (n.x > e.y && n.x > d.z) {
    const float s = 2*sqrtf(1+n.x-e.y-d.z);
    q = {(d.y-e.z)/s,s*.25f,(n.y+e.x)/s,(n.z+d.x)/s};
  } else if (e.y > d.z) {
    const float s = 2*sqrtf(1+e.y-n.x-d.z);
    q = {(n.z-d.x)/s,(n.y+e.x)/s,s*.25f,(e.z+d.y)/s};
  } else {
    const float s = 2*sqrtf(1+d.z-n.x-e.y);
    q = {(e.x-n.y)/s,(n.z+d.x)/s,(e.z+d.y)/s,s*.25f};
  }
  const float l = sqrtf(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
  return {q.w/l,q.x/l,q.y/l,q.z/l};
}

void Tvc::resetReferenceSamples() {
  referenceCount=0; upSum={}; eastSum={};
}

bool Tvc::acquireReference(const ImuSample &s, const char *&waitReason) {
  upSum = plus(upSum, rotate(s.qWB,s.accelB));
  eastSum = plus(eastSum,rotate(s.qWB,{0,1,0}));
  if (++referenceCount < config::referenceSamples) {
    waitReason="HOLD_STILL";
    return false;
  }
  // IMUPLUS 방위각과 독립된 지상 기준. 기체를 세웠을 때의 +Y 투영을 로컬 East로 사용.
  const float upLength=norm(upSum);
  if (upLength < .001f) {
    resetReferenceSamples();
    waitReason="REFERENCE_GEOMETRY";
    return false;
  }
  const Vec d = scale(upSum,-1/upLength);
  const Vec ep = minus(eastSum,scale(d,dot(d,eastSum)));
  if (norm(ep) < .001f) {
    resetReferenceSamples();
    waitReason="REFERENCE_GEOMETRY";
    return false;
  }
  const Vec e = scale(ep,1/norm(ep));
  qNW = matrixToQuat(cross(e,d),e,d);
  return true;
}

Vec Tvc::updateController(const ImuSample &s, Quat currentQ, bool integrate) {
  if (!previousSample) { qPrev=currentQ; gyroPrev=s.gyroB; previousSample=true; }
  if (!integrate) { velocity={}; accFiltered={}; }
  const float n = -model::kvNorth*velocity.x-model::kaNorth*accFiltered.x;
  const float e = -model::kvEast*velocity.y-model::kaEast*accFiltered.y;
  const float size = hypotf(n,e);
  Quat target = uprightQ;
  if (size > 1e-9f) {
    const float half = .5f*config::rad*fminf(size,model::targetTiltDeg);
    const float factor = sinf(half)/size;
    target = multiply({cosf(half),e*factor,-n*factor,0},uprightQ);
  }
  Quat error = multiply(conj(qPrev),target);
  if (error.w < 0) error={-error.w,-error.x,-error.y,-error.z};
  const Vec axis{error.x,error.y,error.z};
  const float magnitude = norm(axis);
  const float gain = magnitude < 1e-8f ? 2.0f : 2*atan2f(magnitude,error.w)/magnitude;
  const Vec alpha = minus(scale(axis,model::kpAtt*gain),scale(gyroPrev,model::kdAtt));
  if (integrate) {
    const Vec a = plus(rotate(currentQ,s.accelB),{0,0,model::g});
    velocity = plus(velocity,scale(a,model::dt));
    const float beta = 1-expf(-model::dt/model::tauAcc);
    accFiltered = plus(accFiltered,scale(minus(a,accFiltered),beta));
  }
  qPrev = currentQ; gyroPrev = s.gyroB;
  return alpha;
}

Command Tvc::allocate(Vec alpha, Quat q, Vec omega, float totalN, float cap1, float cap2) const {
  const float total = clamp(totalN,0,cap1+cap2);
  if (total <= 0) return {};
  const Quat qRel = multiply(conj(uprightQ),q);
  float roll = 0;
  if (hypotf(qRel.w,qRel.x) > 1e-9f) {
    const float angle = 2*atan2f(qRel.x,qRel.w);
    roll = atan2f(sinf(angle),cosf(angle));
  }
  alpha.x = -model::kpRoll*roll-model::kdRoll*omega.x;
  const Vec wanted = plus(alpha,applyJinv(cross(omega,applyJ(omega))));
  const float lo = fmaxf(0,total-cap2), hi = fminf(cap1,total);
  int bestA=0,bestB=0;
  float bestCost=INFINITY,bestT1=total*.5f;
  for (int stage=0,step=grid/4; stage<8; ++stage,step/=2) {
    const int centerA=bestA,centerB=bestB,radius=stage==0 ? 4 : 1;
    for (int i=-radius;i<=radius;++i) {
      const int a = constrain(centerA+i*step,-grid,grid);
      const float ca=cosine[a+grid],sa=sine[a+grid];
      const auto ry = [ca,sa](Vec v) { return Vec{ca*v.x+sa*v.z,v.y,-sa*v.x+ca*v.z}; };
      const Vec outerCenter=plus(pivot,ry(link1));
      for (int j=-radius;j<=radius;++j) {
        const int b = constrain(centerB+j*step,-grid,grid);
        const float cb=cosine[b+grid],sb=sine[b+grid];
        const Vec dir=ry({cb,sb,0});
        const Vec pos=plus(outerCenter,ry({cb*link2.x-sb*link2.y,sb*link2.x+cb*link2.y,link2.z}));
        const Vec tvc=applyJinv(cross(pos,scale(dir,total))), reaction=applyJinv(dir);
        const float qWanted=dot(reaction,minus(wanted,tvc))/dot(reaction,reaction);
        const float t1=clamp((model::kq2*total-qWanted)/(model::kq1+model::kq2),lo,hi);
        const float moment=model::kq2*(total-t1)-model::kq1*t1;
        const Vec error=minus(plus(tvc,scale(reaction,moment)),wanted);
        const float cost=dot(error,error);
        if (cost<bestCost) { bestCost=cost;bestA=a;bestB=b;bestT1=t1; }
      }
    }
  }
  return {bestT1,total-bestT1,bestA*model::maxServoDeg/grid,bestB*model::maxServoDeg/grid};
}

template <size_t N>
bool Tvc::validCurve(const config::ThrustPoint (&curve)[N]) {
  if (N < 2 || curve[0].newtons != 0 || curve[0].microseconds != config::escStopUs) return false;
  for (size_t i=1;i<N;++i) {
    if (!isfinite(curve[i].newtons) || curve[i].newtons <= curve[i-1].newtons ||
        curve[i].microseconds <= curve[i-1].microseconds || curve[i].microseconds > config::escMaxUs) return false;
  }
  return true;
}

template <size_t N>
int Tvc::thrustPulse(const config::ThrustPoint (&curve)[N], float n) {
  if (n <= 0) return config::escStopUs;
  for (size_t i=1;i<N;++i) {
    if (n <= curve[i].newtons) {
      const auto &a=curve[i-1], &b=curve[i];
      const float u=(n-a.newtons)/(b.newtons-a.newtons);
      return lroundf(a.microseconds+u*(b.microseconds-a.microseconds));
    }
  }
  return curve[N-1].microseconds;
}

void Tvc::startEsc(uint32_t now) {
  // Servo ISR가 attach 직후의 기본값을 읽기 전에 두 정지 펄스를 설정한다.
  noInterrupts();
  escA.attach(config::pinEscA,config::escStopUs,config::escMaxUs);
  escB.attach(config::pinEscB,config::escStopUs,config::escMaxUs);
  escA.writeMicroseconds(config::escStopUs);
  escB.writeMicroseconds(config::escStopUs);
  interrupts();
  if (!escA.attached() || !escB.attached()) { disarm(); return; }
  escAPulse=escBPulse=config::escStopUs;
  escAt=now;
  state=ArmState::Initializing;
}

void Tvc::disarm() {
  noInterrupts();
  if (escA.attached()) escA.detach();
  if (escB.attached()) escB.detach();
  pinMode(config::pinEscA,OUTPUT); digitalWrite(config::pinEscA,LOW);
  pinMode(config::pinEscB,OUTPUT); digitalWrite(config::pinEscB,LOW);
  interrupts();
  escAPulse=escBPulse=0;
  benchPulseUs=config::escStopUs;
  benchEsc=BenchEsc::None;
  serialLength=0;
  serialOverflow=false;
  fan1Applied=fan2Applied=0;
  velocity={}; accFiltered={};
  state=ArmState::Disarmed;
  offSeen=false;
}

void Tvc::writeServo(float a, float b, float maximumDeg) {
  j1Applied=slew(j1Applied,clamp(a,-maximumDeg,maximumDeg),model::servoRateDegS,model::servoRateDegS);
  j2Applied=slew(j2Applied,clamp(b,-maximumDeg,maximumDeg),model::servoRateDegS,model::servoRateDegS);
  servo1.writeMicroseconds(constrain(int(lroundf(config::j1CenterUs+config::j1PwmSign*config::j1UsPerDeg*j1Applied)),1000,2000));
  servo2.writeMicroseconds(constrain(int(lroundf(config::j2CenterUs+config::j2PwmSign*config::j2UsPerDeg*j2Applied)),1000,2000));
}

const char *Tvc::stateName(ArmState armState) {
  switch (armState) {
    case ArmState::Disarmed: return "DISARMED";
    case ArmState::Initializing: return "ESC_INIT";
    case ArmState::Idle: return "ARMED_IDLE";
    case ArmState::Active: return "ARMED_ACTIVE";
    case ArmState::Bench: return "ARMED_BENCH";
  }
  return "UNKNOWN";
}

void Tvc::log(const char *status, RcInput rc, float tiltDeg, float total, Command u) {
  const uint32_t now=micros();
  if (now-lastLog < 250000) return;
  lastLog=now;
  constexpr bool use8040=config::referenceProp==config::ReferenceProp::Gws8040;
  char tiltText[16];
  if (isfinite(tiltDeg)) snprintf(tiltText,sizeof(tiltText),"%.1f",tiltDeg);
  else strcpy(tiltText,"NA");
  char buffer[304];
  const int n=snprintf(buffer,sizeof(buffer),
      "%s %s | RC=%d THR=%d ARM=%d TILT=%s | SERVO_OK=%d ORDER_OK=%d CURVE_OK=%d,%d MAP=%s | T=%.2f REQ=%.2f,%.2f J=%.1f,%.1f PWM=%d,%d | REF3S=%s@%.1fV REF_N=%.2f EST_US=%d,%d\n",
      stateName(state),status,rc.valid,rc.throttleUs,rc.armUs,tiltText,
      config::servoGeometryConfirmed,config::fanOrderConfirmed,curve1Valid,curve2Valid,
      config::usePublished3sEstimate?"EST_3S":"MEASURED",total,
      u.fan1,u.fan2,j1Applied,j2Applied,escAPulse,escBPulse,
      use8040?"8040":"9050",config::referenceLoadedVolts,referencePairN,
      reference3s::estimatedPulse(u.fan1,referencePairN),reference3s::estimatedPulse(u.fan2,referencePairN));
  if (n>0 && n<int(sizeof(buffer)) && Serial.availableForWrite()>=n)
    Serial.write(reinterpret_cast<const uint8_t*>(buffer),size_t(n));
}

void Tvc::logWaitUpright(RcInput rc, ImuSample s, UprightCheck check, const char *reason) {
  const uint32_t now=micros();
  if (now-lastLog<250000) return;
  lastLog=now;
  char tiltText[16];
  if (isfinite(check.upCos)) {
    snprintf(tiltText,sizeof(tiltText),"%.1f",acosf(clamp(check.upCos,-1,1))/config::rad);
  } else {
    strcpy(tiltText,"NA");
  }
  char buffer[192];
  const int n=snprintf(buffer,sizeof(buffer),
      "DISARMED WAIT_UPRIGHT WHY=%s THR=%d ARM=%d TILT_ACC=%s A_B=%.2f,%.2f,%.2f A_NORM=%.2f GYRO_DPS=%.2f STILL=%u/%u\n",
      reason,rc.throttleUs,rc.armUs,tiltText,s.accelB.x,s.accelB.y,s.accelB.z,
      check.accelNorm,check.gyroNorm/config::rad,referenceCount,config::referenceSamples);
  if (n>0 && n<int(sizeof(buffer)) && Serial.availableForWrite()>=n)
    Serial.write(reinterpret_cast<const uint8_t*>(buffer),size_t(n));
}

void Tvc::setup() {
  pinMode(config::pinEscA,OUTPUT); digitalWrite(config::pinEscA,LOW);
  pinMode(config::pinEscB,OUTPUT); digitalWrite(config::pinEscB,LOW);
  Serial.begin(115200);
  noInterrupts();
  servo1.attach(config::pinJ1,1000,2000); servo1.writeMicroseconds(config::j1CenterUs);
  servo2.attach(config::pinJ2,1000,2000); servo2.writeMicroseconds(config::j2CenterUs);
  interrupts();
  if (config::useSbus) Serial1.begin(100000,SERIAL_8E2_RXINV);
  else {
    pinMode(config::pinThrottle,INPUT); pinMode(config::pinArm,INPUT);
    attachInterrupt(digitalPinToInterrupt(config::pinThrottle),throttleIsr,CHANGE);
    attachInterrupt(digitalPinToInterrupt(config::pinArm),armIsr,CHANGE);
  }
  Wire.begin();
  Wire.setClock(100000);
  imuStarted=bno.begin(OPERATION_MODE_IMUPLUS);
  if (imuStarted) {
    // AXIS_MAP_CONFIG=0x24, AXIS_MAP_SIGN=0: 보드에서 확인한 원래 축 기준.
    // 라이브러리 시작 후 아직 다르면 잠근다. UNIT_SEL 하위 2비트는 m/s² 및 deg/s.
    uint8_t remap[2]{}, unit=0;
    imuStarted=readRegisters(0x41,remap,2) && readRegisters(0x3B,&unit,1)
            && remap[0]==0x24 && remap[1]==0 && (unit&3)==0;
  }
  for (int k=-grid;k<=grid;++k) {
    const float angle=k*model::maxServoDeg*config::rad/grid;
    cosine[k+grid]=cosf(angle); sine[k+grid]=sinf(angle);
  }
  referencePairN=config::referenceProp==config::ReferenceProp::Gws8040
      ? reference3s::totalAt(reference3s::gws8040,config::referenceLoadedVolts)
      : reference3s::totalAt(reference3s::gws9050,config::referenceLoadedVolts);
  if (config::usePublished3sEstimate) {
    curve1Valid=curve2Valid=referencePairN>0;
    fan1Cap=fminf(model::tFan1Max,0.5f*referencePairN);
    fan2Cap=fminf(model::tFan2Max,0.5f*referencePairN);
  } else {
    curve1Valid=validCurve(config::fan1Curve);
    curve2Valid=validCurve(config::fan2Curve);
    constexpr size_t n1 = sizeof(config::fan1Curve)/sizeof(config::fan1Curve[0]);
    constexpr size_t n2 = sizeof(config::fan2Curve)/sizeof(config::fan2Curve[0]);
    fan1Cap=fminf(model::tFan1Max,config::fan1Curve[n1-1].newtons);
    fan2Cap=fminf(model::tFan2Max,config::fan2Curve[n2-1].newtons);
  }
  motorReady=config::servoGeometryConfirmed && config::fanOrderConfirmed && curve1Valid && curve2Valid;
  nextTick=micros();
}

void Tvc::loop() {
  serviceSbus();
  uint32_t now=micros();
  const RcInput initial=readRc();
  if (state!=ArmState::Disarmed && (!initial.valid || initial.armUs>config::armOffUs)) disarm();
  serviceBenchSerial(referenceReady && state==ArmState::Disarmed && initial.valid
      && initial.armUs>config::armOffUs && initial.throttleUs>=1000 && initial.throttleUs<=config::throttleLowUs);
  if (int32_t(now-nextTick)<0) return;
  nextTick=now+config::periodUs;
  const bool late=lastTick && now-lastTick>config::maxGapUs;
  lastTick=now;
  ImuSample sample{};
  const bool imuOk=imuStarted && readImu(sample);
  serviceSbus();
  const RcInput rc=readRc();
  now=micros();
  if (!imuOk || late || !rc.valid || now-lastTick>=config::maxGapUs) {
    disarm();
    resetReferenceSamples(); previousSample=false;
    writeServo(0,0,config::previewLimitDeg);
    log(!imuOk?"IMU_OR_AXIS":late?"LOOP_LATE":"RC_OR_TIMING",rc,NAN,0,{});
    return;
  }
  const bool off=rc.armUs>config::armOffUs;
  const bool on=rc.armUs<config::armOnUs;
  const bool low=rc.throttleUs>=1000 && rc.throttleUs<=config::throttleLowUs;
  if (!referenceReady) {
    const float accelNorm=norm(sample.accelB);
    const UprightCheck check{accelNorm,norm(sample.gyroB),accelNorm>0 ? sample.accelB.x/accelNorm : NAN};
    const char *reason=nullptr;
    if (!off) reason="ARM_NOT_OFF";
    else if (!low) reason="THROTTLE_NOT_LOW";
    else if (!(check.accelNorm>config::referenceMinAccel && check.accelNorm<config::referenceMaxAccel))
      reason="ACCEL_RANGE";
    else if (!(check.upCos>config::referenceMinUpCos)) reason="BODY_X_NOT_UP";
    else if (!(check.gyroNorm<config::referenceMaxGyro)) reason="GYRO_MOVING";
    if (reason) resetReferenceSamples();
    else referenceReady=acquireReference(sample,reason);
    if (!referenceReady) {
      logWaitUpright(rc,sample,check,reason);
      return;
    }
  }
  const Quat attitude=multiply(qNW,sample.qWB);
  const float tilt=acosf(clamp(-rotate(attitude,{1,0,0}).z,-1,1))/config::rad;
  if (tilt>config::tiltCutoffDeg) {
    disarm(); previousSample=false;
    writeServo(0,0,config::previewLimitDeg);
    log("TILT_LIMIT",rc,tilt,0,{});
    return;
  }
  if (off) {
    if (state!=ArmState::Disarmed) disarm();
    offSeen=true;
  } else if (on && state==ArmState::Disarmed && offSeen) {
    offSeen=false;
    if (low) startEsc(now);
  }
  if (state==ArmState::Initializing) {
    if (!low) disarm();
    else if (now-escAt>=config::escInitUs) state=ArmState::Idle;
  }
  const float stick=clamp(float(rc.throttleUs-config::throttleLowUs)/(2000-config::throttleLowUs),0,1);
  const bool benchSelected=benchEsc!=BenchEsc::None;
  const float total=benchSelected?0:stick*(motorReady?fminf(model::tPeak,fan1Cap+fan2Cap):model::tPeak);
  if (state==ArmState::Idle || state==ArmState::Active || state==ArmState::Bench) {
    if (benchSelected && stick>0) state=ArmState::Bench;
    else state=motorReady && total>0 ? ArmState::Active : ArmState::Idle;
  }
  const bool drive=state==ArmState::Active;
  const Vec alpha=updateController(sample,attitude,drive);
  // ESC가 잠겨도 서보 시험용으로 모델의 mg에서 배분을 계산한다.
  const Command cmd=allocate(alpha,attitude,sample.gyroB,
      drive ? total : model::mass*model::g,
      drive ? fan1Cap : model::tFan1Max, drive ? fan2Cap : model::tFan2Max);
  writeServo(cmd.j1,cmd.j2,drive ? model::maxServoDeg : config::previewLimitDeg);
  if (state!=ArmState::Disarmed) {
    if (drive) {
      fan1Applied=slew(fan1Applied,cmd.fan1,model::fanRiseNs,model::fanFallNs);
      fan2Applied=slew(fan2Applied,cmd.fan2,model::fanRiseNs,model::fanFallNs);
      const int f1=config::usePublished3sEstimate ? reference3s::estimatedPulse(fan1Applied,referencePairN)
                                                 : thrustPulse(config::fan1Curve,fan1Applied);
      const int f2=config::usePublished3sEstimate ? reference3s::estimatedPulse(fan2Applied,referencePairN)
                                                 : thrustPulse(config::fan2Curve,fan2Applied);
      escAPulse=config::escAIsModelFan1 ? f1 : f2;
      escBPulse=config::escAIsModelFan1 ? f2 : f1;
    } else if (state==ArmState::Bench) {
      const int target=config::escStopUs+lroundf(stick*(config::benchMaxUs-config::escStopUs));
      benchPulseUs=constrain(target,config::escStopUs,benchPulseUs+config::benchRisePerTickUs);
      escAPulse=benchEsc==BenchEsc::A ? benchPulseUs : config::escStopUs;
      escBPulse=benchEsc==BenchEsc::B ? benchPulseUs : config::escStopUs;
    } else {
      fan1Applied=fan2Applied=0;
      benchPulseUs=config::escStopUs;
      escAPulse=escBPulse=config::escStopUs;
    }
    escA.writeMicroseconds(escAPulse); escB.writeMicroseconds(escBPulse);
  }
  const char *status=benchSelected ? (benchEsc==BenchEsc::A?"BENCH_A":"BENCH_B")
      : (motorReady?"OK":"CAL_REQUIRED");
  log(status,rc,tilt,total,cmd);
}

Tvc controller;
void setup() { controller.setup(); }
void loop() { controller.loop(); }
