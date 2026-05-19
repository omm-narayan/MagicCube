/*
 * ══════════════════════════════════════════════════════════════
 *  MagicCube — Advanced Multi-Sensor Dashboard  v2.1
 *  Connect to WiFi "MagicCube" (no password)
 *  Open browser → http://192.168.4.1
 * ══════════════════════════════════════════════════════════════
 *  BOARD : ESP32S3 Dev Module
 *  SDA   = GPIO 8   |   SCL = GPIO 9   |   XSHUT = GPIO 3
 *
 *  SENSORS (all share I²C bus):
 *  BME688  : SDO→GND, CS→3V3   → addr 0x76
 *            Temperature · Humidity · Pressure · Gas Resistance
 *            Altitude · Dew Point · Heat Index (derived)
 *  BNO055  : ADR→3V3            → addr 0x28
 *            Accel X/Y/Z · Gyro X/Y/Z · Mag X/Y/Z
 *            Euler Roll/Pitch/Yaw · Quaternion W/X/Y/Z
 *  VL53L1X : XSHUT→GPIO 3       → addr 0x29
 *            Distance (mm) · Velocity m/s (filtered)
 *  TSL2561 : ADDR→float          → addr 0x39
 *            Ambient Light (lux)
 *
 *  LIBRARIES (Sketch → Library Manager):
 *    Adafruit BME680 Library
 *    Adafruit BNO055
 *    Adafruit Unified Sensor
 *    VL53L1X by Pololu
 *    Adafruit TSL2561
 * ══════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BNO055.h>
#include <VL53L1X.h>
#include <Adafruit_TSL2561_U.h>
#include <math.h>

#define SDA_PIN       8
#define SCL_PIN       9
#define XSHUT_PIN     3
#define SEA_LEVEL_HPA 1013.25f

// Velocity filtering parameters
#define VEL_ALPHA     0.25f     // Smoothing factor (0.1-0.5, lower = smoother)
#define MAX_VEL_MS    5.0f      // Max realistic velocity (m/s) ±5 m/s
#define MIN_DT        0.02f     // Minimum time delta (20ms) to avoid division by zero
#define MAX_VEL_JUMP  3.0f      // Max velocity jump per sample (m/s) for noise rejection

Adafruit_BME680          bme;
Adafruit_BNO055          bno(55, 0x28);
VL53L1X                  tof;
Adafruit_TSL2561_Unified tsl(TSL2561_ADDR_FLOAT, 12345);
WebServer                server(80);

bool bme_ok=false, bno_ok=false, tof_ok=false, tsl_ok=false;

struct SensorData {
  float temp=0, hum=0, pres=0, gas=0, alt=0, dew=0, heatIdx=0;
  float roll=0, pitch=0, yaw=0;
  float ax=0, ay=0, az=0;
  float gx=0, gy=0, gz=0;
  float mx=0, my=0, mz=0;
  float qw=1, qx=0, qy=0, qz=0;
  int   dist=0;
  float vel=0;        // Now in m/s (stabilized)
  float lux=0;
} D;

unsigned long lastTofMs=0;
int           lastTofDist=0;
float         velFiltered=0;
float         lastVelRaw=0;

float calcAltitude(float hPa){
  return 44330.0f*(1.0f-pow(hPa/SEA_LEVEL_HPA,0.1903f));
}

float calcDewPoint(float t,float rh){
  float a=17.27f,b=237.3f;
  float al=((a*t)/(b+t))+log(rh/100.0f);
  return (b*al)/(a-al);
}

float calcHeatIndex(float tc,float rh){
  float T=tc*9.0f/5.0f+32.0f;
  float HI=-42.379f+2.04901523f*T+10.14333127f*rh
           -0.22475541f*T*rh-6.83783e-3f*T*T
           -5.481717e-2f*rh*rh+1.22874e-3f*T*T*rh
           +8.5282e-4f*T*rh*rh-1.99e-6f*T*T*rh*rh;
  return (HI-32.0f)*5.0f/9.0f;
}

void readAll(){
  if(bme_ok && bme.performReading()){
    D.temp=bme.temperature; D.hum=bme.humidity;
    D.pres=bme.pressure/100.0f; D.gas=bme.gas_resistance/1000.0f;
    D.alt=calcAltitude(D.pres);
    D.dew=calcDewPoint(D.temp,D.hum);
    D.heatIdx=calcHeatIndex(D.temp,D.hum);
  }
  
  if(bno_ok){
    sensors_event_t e;
    bno.getEvent(&e,Adafruit_BNO055::VECTOR_EULER);
    D.yaw=e.orientation.x; D.pitch=e.orientation.y; D.roll=e.orientation.z;
    bno.getEvent(&e,Adafruit_BNO055::VECTOR_ACCELEROMETER);
    D.ax=e.acceleration.x; D.ay=e.acceleration.y; D.az=e.acceleration.z;
    bno.getEvent(&e,Adafruit_BNO055::VECTOR_GYROSCOPE);
    D.gx=e.gyro.x; D.gy=e.gyro.y; D.gz=e.gyro.z;
    bno.getEvent(&e,Adafruit_BNO055::VECTOR_MAGNETOMETER);
    D.mx=e.magnetic.x; D.my=e.magnetic.y; D.mz=e.magnetic.z;
    imu::Quaternion q=bno.getQuat();
    D.qw=q.w(); D.qx=q.x(); D.qy=q.y(); D.qz=q.z();
  }
  
  // STABILIZED VELOCITY CALCULATION (m/s)
  if(tof_ok){
    tof.read(false);
    int nd = tof.ranging_data.range_mm;
    unsigned long now = millis();
    
    if(lastTofMs > 0 && nd > 0) { 
      float dt = (now - lastTofMs) / 1000.0f;
      
      if(dt > MIN_DT) {
        // Raw velocity in mm/s, convert to m/s
        float rawVelMs = ((float)(nd - lastTofDist) / dt) / 1000.0f;
        
        // Noise rejection: ignore physically impossible jumps
        bool validReading = (fabs(rawVelMs - lastVelRaw) < MAX_VEL_JUMP) && 
                            (fabs(rawVelMs) < MAX_VEL_MS * 2);
        
        if(validReading) {
          // Exponential moving average filter for smooth velocity
          if(fabs(velFiltered) < 0.001f) {
            velFiltered = rawVelMs;  // First reading
          } else {
            velFiltered = VEL_ALPHA * rawVelMs + (1.0f - VEL_ALPHA) * velFiltered;
          }
          
          // Clamp to realistic range
          velFiltered = constrain(velFiltered, -MAX_VEL_MS, MAX_VEL_MS);
          lastVelRaw = rawVelMs;
        }
      }
    }
    
    lastTofDist = nd;
    lastTofMs = now;
    D.dist = nd;
    D.vel = velFiltered;  // Already in m/s
  }
  
  if(tsl_ok){ 
    sensors_event_t e; 
    tsl.getEvent(&e); 
    if(e.light>=0) D.lux=e.light; 
  }
}

void handleData(){
  readAll();
  char buf[1400];
  snprintf(buf,sizeof(buf),
    "{\"bme\":%s,\"bno\":%s,\"tof\":%s,\"tsl\":%s,"
    "\"temp\":%.1f,\"hum\":%.1f,\"pres\":%.1f,\"gas\":%.1f,\"alt\":%.1f,\"dew\":%.1f,\"heatIdx\":%.1f,"
    "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"mx\":%.2f,\"my\":%.2f,\"mz\":%.2f,"
    "\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,"
    "\"dist\":%d,\"vel\":%.3f,\"lux\":%.1f}",
    bme_ok?"true":"false",bno_ok?"true":"false",
    tof_ok?"true":"false",tsl_ok?"true":"false",
    D.temp,D.hum,D.pres,D.gas,D.alt,D.dew,D.heatIdx,
    D.roll,D.pitch,D.yaw,
    D.ax,D.ay,D.az,D.gx,D.gy,D.gz,D.mx,D.my,D.mz,
    D.qw,D.qx,D.qy,D.qz,D.dist,D.vel,D.lux);
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.send(200,"application/json",buf);
}

void handleRoot(){
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/html","");

  server.sendContent(F(R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MagicCube</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=DM+Sans:ital,opsz,wght@0,9..40,300;0,9..40,400;0,9..40,500;1,9..40,300&family=DM+Mono:wght@300;400&display=swap">
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
:root{
  --bg:#f5f5f7;
  --surface:#ffffff;
  --border:#e4e4e7;
  --border-inner:#f0f0f2;
  --text:#1d1d1f;
  --sub:#6e6e73;
  --muted:#aeaeb2;
  --accent:#0071e3;
  --green:#34c759;
  --red:#ff3b30;
  --orange:#ff9f0a;
  --purple:#5e5ce6;
  --teal:#32ade6;
  --pink:#ff375f;
  --r:14px;
}
*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:'DM Sans',sans-serif;
  background:var(--bg);color:var(--text);
  -webkit-font-smoothing:antialiased;
  min-height:100vh;
}

/* HEADER */
header{
  position:sticky;top:0;z-index:50;
  display:flex;align-items:center;justify-content:space-between;
  padding:13px 24px;
  background:rgba(245,245,247,.9);
  backdrop-filter:saturate(180%) blur(18px);
  -webkit-backdrop-filter:saturate(180%) blur(18px);
  border-bottom:1px solid var(--border);
}
.brand{display:flex;align-items:baseline;gap:8px}
.brand-name{font-size:1.1rem;font-weight:500;letter-spacing:-.015em}
.brand-ver{font-size:.68rem;color:var(--muted);font-weight:300;letter-spacing:.02em}
#ts{font-size:.7rem;color:var(--sub);font-family:'DM Mono',monospace;font-weight:300}

/* CHIP BAR */
.chipbar{
  display:flex;gap:7px;padding:9px 24px;
  background:var(--surface);
  border-bottom:1px solid var(--border);
  flex-wrap:wrap;
}
.chip{
  display:inline-flex;align-items:center;gap:5px;
  padding:3px 10px 3px 8px;border-radius:20px;
  font-size:.65rem;font-weight:400;letter-spacing:.04em;
  border:1px solid var(--border);color:var(--muted);background:transparent;
  transition:all .2s;
}
.chip.on{background:#e8f4fd;border-color:#b8d9f8;color:var(--accent)}
.cdot{width:5px;height:5px;border-radius:50%;background:var(--muted);flex-shrink:0;transition:background .2s}
.chip.on .cdot{background:var(--green)}

/* LAYOUT */
.page{padding:20px 24px 32px;display:flex;flex-direction:column;gap:26px}
.sec-label{
  font-size:.62rem;font-weight:500;text-transform:uppercase;
  letter-spacing:.1em;color:var(--muted);
  padding-bottom:9px;border-bottom:1px solid var(--border-inner);
  margin-bottom:2px;
}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(288px,1fr));gap:11px}

/* CARD */
.card{
  background:var(--surface);
  border:1px solid var(--border);
  border-radius:var(--r);
  padding:17px 17px 13px;
}
.card-lbl{
  font-size:.6rem;font-weight:500;text-transform:uppercase;
  letter-spacing:.1em;color:var(--muted);
  display:flex;align-items:center;justify-content:space-between;
  margin-bottom:11px;
}
.tag{
  font-size:.59rem;font-weight:400;letter-spacing:.02em;
  background:var(--bg);border:1px solid var(--border);
  color:var(--sub);padding:1px 7px;border-radius:8px;
  text-transform:none;
}

/* BIG NUMBER */
.big{
  font-size:2.25rem;font-weight:300;letter-spacing:-.025em;
  line-height:1.05;color:var(--text);margin-bottom:2px;
}
.big .u{font-size:.95rem;font-weight:300;color:var(--muted);margin-left:3px}
.hint{font-size:.7rem;color:var(--muted);margin-bottom:11px;font-weight:300;font-style:italic}

/* METRIC CELLS */
.row{display:grid;gap:7px;margin-bottom:9px}
.c2{grid-template-columns:1fr 1fr}
.c3{grid-template-columns:1fr 1fr 1fr}
.c4{grid-template-columns:1fr 1fr 1fr 1fr}
.cell{
  background:var(--bg);border-radius:9px;
  padding:8px 6px 7px;text-align:center;
  border:1px solid var(--border-inner);
}
.cl{font-size:.57rem;text-transform:uppercase;letter-spacing:.07em;color:var(--muted);margin-bottom:3px}
.cv{font-size:.88rem;font-weight:400;font-family:'DM Mono',monospace;color:var(--text)}
.cv.ac{color:var(--accent)} .cv.gr{color:var(--green)}
.cv.or{color:var(--orange)} .cv.pu{color:var(--purple)}
.cv.te{color:var(--teal)}   .cv.pk{color:var(--pink)}

/* CHART */
.ch{position:relative;width:100%}
.ch.h70{height:70px} .ch.h86{height:86px} .ch.h96{height:96px}

/* 3D CUBE */
.scene{display:flex;justify-content:center;align-items:center;height:96px;perspective:260px;margin-bottom:8px}
.cube{width:50px;height:50px;transform-style:preserve-3d;transition:transform .08s linear}
.face{
  position:absolute;width:50px;height:50px;
  background:rgba(0,113,227,.03);
  border:1px solid rgba(0,113,227,.15);
  display:flex;align-items:center;justify-content:center;
  font-size:.42rem;color:rgba(0,113,227,.35);letter-spacing:.08em;
  font-family:'DM Mono',monospace;
}
.f-fr{transform:translateZ(25px)} .f-bk{transform:rotateY(180deg) translateZ(25px)}
.f-lt{transform:rotateY(-90deg) translateZ(25px)} .f-rt{transform:rotateY(90deg) translateZ(25px)}
.f-tp{transform:rotateX(90deg) translateZ(25px)} .f-bt{transform:rotateX(-90deg) translateZ(25px)}

/* OFFLINE */
.off{padding:20px 0;text-align:center;color:var(--muted);font-size:.75rem;font-weight:300}

footer{
  padding:18px 24px;text-align:center;
  font-size:.63rem;color:var(--muted);letter-spacing:.04em;
  border-top:1px solid var(--border-inner);
}

@media(max-width:540px){
  .page{padding:13px 13px 24px}
  header,.chipbar{padding-left:13px;padding-right:13px}
  .big{font-size:1.8rem}
  .c4{grid-template-columns:1fr 1fr}
}
</style>
</head>
<body>

<header>
  <div class="brand">
    <span class="brand-name">MagicCube</span>
    <span class="brand-ver">Sensor Dashboard</span>
  </div>
  <div id="ts">Connecting…</div>
</header>

<div class="chipbar">
  <div class="chip" id="chip-bme"><span class="cdot"></span>BME688</div>
  <div class="chip" id="chip-bno"><span class="cdot"></span>BNO055</div>
  <div class="chip" id="chip-tof"><span class="cdot"></span>VL53L1X</div>
  <div class="chip" id="chip-tsl"><span class="cdot"></span>TSL2561</div>
</div>

<div class="page">

  <section>
    <div class="sec-label">Environment · BME688</div>
    <div class="grid">
      <div class="card" id="cd-temp"><div class="card-lbl">Temperature<span class="tag">°C</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-hum"><div class="card-lbl">Humidity &amp; Comfort<span class="tag">% RH</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-pres"><div class="card-lbl">Pressure &amp; Altitude<span class="tag">hPa · m</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-gas"><div class="card-lbl">Gas Resistance<span class="tag">kΩ</span></div><div class="off">Not connected</div></div>
    </div>
  </section>

  <section>
    <div class="sec-label">Motion &amp; Orientation · BNO055</div>
    <div class="grid">
      <div class="card" id="cd-euler"><div class="card-lbl">Euler Orientation<span class="tag">°</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-accel"><div class="card-lbl">Accelerometer<span class="tag">m/s²</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-gyro"><div class="card-lbl">Gyroscope<span class="tag">°/s</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-mag"><div class="card-lbl">Magnetometer<span class="tag">µT</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-quat"><div class="card-lbl">Quaternion &amp; 3D Attitude</div><div class="off">Not connected</div></div>
    </div>
  </section>

  <section>
    <div class="sec-label">Distance &amp; Velocity · VL53L1X</div>
    <div class="grid">
      <div class="card" id="cd-dist"><div class="card-lbl">Distance<span class="tag">mm</span></div><div class="off">Not connected</div></div>
      <div class="card" id="cd-vel"><div class="card-lbl">Velocity (filtered)<span class="tag">m/s</span></div><div class="off">Not connected</div></div>
    </div>
  </section>

  <section>
    <div class="sec-label">Ambient Light · TSL2561</div>
    <div class="grid">
      <div class="card" id="cd-lux"><div class="card-lbl">Light Intensity<span class="tag">lux</span></div><div class="off">Not connected</div></div>
    </div>
  </section>

</div>

<footer>MagicCube v2.1 &nbsp;·&nbsp; ESP32-S3 &nbsp;·&nbsp; BME688 · BNO055 · VL53L1X · TSL2561 &nbsp;·&nbsp; Velocity filtered & stabilized in m/s</footer>
)HTML"));

  server.sendContent(F(R"JS(
<script>
const MAX=55,charts={},init={};

const C={
  blue:'#0071e3',green:'#34c759',orange:'#ff9f0a',
  purple:'#5e5ce6',teal:'#32ade6',pink:'#ff375f',
  amber:'#ffcc00',slate:'#8e8e93'
};

function mkChart(id,sets,o){
  const el=document.getElementById(id); if(!el)return null; o=o||{};
  return new Chart(el,{type:'line',
    data:{
      labels:Array(MAX).fill(''),
      datasets:sets.map(s=>({
        label:s.l,data:Array(MAX).fill(null),
        borderColor:s.c,backgroundColor:s.c+'12',
        borderWidth:1.5,pointRadius:0,
        fill:sets.length===1,tension:0.38
      }))
    },
    options:{
      animation:false,responsive:true,maintainAspectRatio:false,
      plugins:{legend:{
        display:sets.length>1,
        labels:{font:{family:'DM Mono',size:9},color:'#aeaeb2',boxWidth:8,padding:7}
      }},
      scales:{
        x:{display:false},
        y:{display:true,
          grid:{color:'#f0f0f2'},border:{color:'#f0f0f2'},
          ticks:{font:{family:'DM Mono',size:9},color:'#aeaeb2',maxTicksLimit:4},
          ...(o.min!==undefined?{min:o.min}:{}),
          ...(o.max!==undefined?{max:o.max}:{})
        }
      }
    }
  });
}

function push(c,...v){
  if(!c)return;
  v.forEach((val,i)=>{
    c.data.datasets[i].data.push(val);
    if(c.data.datasets[i].data.length>MAX) c.data.datasets[i].data.shift();
  });
  c.data.labels.push('');
  if(c.data.labels.length>MAX) c.data.labels.shift();
  c.update('none');
}

function $v(id,v){const e=document.getElementById(id);if(e)e.textContent=v;}

/* ── card builders ── */
function bTemp(){
  document.getElementById('cd-temp').innerHTML=`
    <div class="card-lbl">Temperature<span class="tag">°C</span></div>
    <div class="big"><span id="vt">--</span><span class="u">°C</span></div>
    <div class="hint">Ambient air temperature</div>
    <div class="ch h70"><canvas id="ch-t"></canvas></div>`;
  charts.temp=mkChart('ch-t',[{l:'°C',c:C.orange}]);
}
function bHum(){
  document.getElementById('cd-hum').innerHTML=`
    <div class="card-lbl">Humidity &amp; Comfort<span class="tag">% RH</span></div>
    <div class="big"><span id="vh">--</span><span class="u">%</span></div>
    <div class="row c3">
      <div class="cell"><div class="cl">Dew Point</div><div class="cv ac" id="vdew">--</div></div>
      <div class="cell"><div class="cl">Heat Index</div><div class="cv or" id="vhi">--</div></div>
      <div class="cell"><div class="cl">Feel</div><div class="cv gr" id="vfeel">--</div></div>
    </div>
    <div class="ch h70"><canvas id="ch-h"></canvas></div>`;
  charts.hum=mkChart('ch-h',[{l:'RH %',c:C.teal},{l:'Dew°C',c:C.blue}]);
}
function bPres(){
  document.getElementById('cd-pres').innerHTML=`
    <div class="card-lbl">Pressure &amp; Altitude<span class="tag">hPa · m</span></div>
    <div class="big"><span id="vp">--</span><span class="u">hPa</span></div>
    <div class="row c2">
      <div class="cell"><div class="cl">Altitude</div><div class="cv pu" id="valt">-- m</div></div>
      <div class="cell"><div class="cl">Sea Level</div><div class="cv te">1013 hPa</div></div>
    </div>
    <div class="ch h70"><canvas id="ch-p"></canvas></div>`;
  charts.pres=mkChart('ch-p',[{l:'hPa',c:C.purple},{l:'Alt m',c:C.green}]);
}
function bGas(){
  document.getElementById('cd-gas').innerHTML=`
    <div class="card-lbl">Gas Resistance<span class="tag">kΩ</span></div>
    <div class="big"><span id="vg">--</span><span class="u">kΩ</span></div>
    <div class="hint">Higher = cleaner air</div>
    <div class="ch h86"><canvas id="ch-g"></canvas></div>`;
  charts.gas=mkChart('ch-g',[{l:'kΩ',c:C.green}]);
}
function bEuler(){
  document.getElementById('cd-euler').innerHTML=`
    <div class="card-lbl">Euler Orientation<span class="tag">°</span></div>
    <div class="row c3">
      <div class="cell"><div class="cl">Roll</div><div class="cv gr" id="vroll">--</div></div>
      <div class="cell"><div class="cl">Pitch</div><div class="cv ac" id="vpitch">--</div></div>
      <div class="cell"><div class="cl">Yaw</div><div class="cv or" id="vyaw">--</div></div>
    </div>
    <div class="ch h96"><canvas id="ch-e"></canvas></div>`;
  charts.euler=mkChart('ch-e',[{l:'Roll',c:C.green},{l:'Pitch',c:C.blue},{l:'Yaw',c:C.orange}],{min:-180,max:180});
}
function bAccel(){
  document.getElementById('cd-accel').innerHTML=`
    <div class="card-lbl">Accelerometer<span class="tag">m/s²</span></div>
    <div class="row c3">
      <div class="cell"><div class="cl">X</div><div class="cv gr" id="vax">--</div></div>
      <div class="cell"><div class="cl">Y</div><div class="cv ac" id="vay">--</div></div>
      <div class="cell"><div class="cl">Z</div><div class="cv pk" id="vaz">--</div></div>
    </div>
    <div class="ch h96"><canvas id="ch-a"></canvas></div>`;
  charts.accel=mkChart('ch-a',[{l:'X',c:C.green},{l:'Y',c:C.blue},{l:'Z',c:C.pink}]);
}
function bGyro(){
  document.getElementById('cd-gyro').innerHTML=`
    <div class="card-lbl">Gyroscope<span class="tag">°/s</span></div>
    <div class="row c3">
      <div class="cell"><div class="cl">X</div><div class="cv gr" id="vgx">--</div></div>
      <div class="cell"><div class="cl">Y</div><div class="cv ac" id="vgy">--</div></div>
      <div class="cell"><div class="cl">Z</div><div class="cv pk" id="vgz">--</div></div>
    </div>
    <div class="ch h96"><canvas id="ch-gy"></canvas></div>`;
  charts.gyro=mkChart('ch-gy',[{l:'X',c:C.green},{l:'Y',c:C.blue},{l:'Z',c:C.pink}]);
}
function bMag(){
  document.getElementById('cd-mag').innerHTML=`
    <div class="card-lbl">Magnetometer<span class="tag">µT</span></div>
    <div class="row c3">
      <div class="cell"><div class="cl">X</div><div class="cv gr" id="vmx">--</div></div>
      <div class="cell"><div class="cl">Y</div><div class="cv ac" id="vmy">--</div></div>
      <div class="cell"><div class="cl">Z</div><div class="cv pu" id="vmz">--</div></div>
    </div>
    <div class="ch h96"><canvas id="ch-m"></canvas></div>`;
  charts.mag=mkChart('ch-m',[{l:'X µT',c:C.green},{l:'Y µT',c:C.blue},{l:'Z µT',c:C.purple}]);
}
function bQuat(){
  document.getElementById('cd-quat').innerHTML=`
    <div class="card-lbl">Quaternion &amp; 3D Attitude</div>
    <div style="display:flex;gap:11px;align-items:flex-start;margin-bottom:9px">
      <div class="scene" style="margin:0;flex:0 0 auto;height:86px">
        <div class="cube" id="cube3d">
          <div class="face f-fr">FRONT</div><div class="face f-bk">BACK</div>
          <div class="face f-lt">L</div><div class="face f-rt">R</div>
          <div class="face f-tp">TOP</div><div class="face f-bt">BOT</div>
        </div>
      </div>
      <div style="flex:1;min-width:0">
        <div class="row c2" style="margin-bottom:6px">
          <div class="cell"><div class="cl">W</div><div class="cv ac" id="vqw">--</div></div>
          <div class="cell"><div class="cl">X</div><div class="cv gr" id="vqx">--</div></div>
        </div>
        <div class="row c2">
          <div class="cell"><div class="cl">Y</div><div class="cv pk" id="vqy">--</div></div>
          <div class="cell"><div class="cl">Z</div><div class="cv pu" id="vqz">--</div></div>
        </div>
      </div>
    </div>
    <div class="ch h70"><canvas id="ch-q"></canvas></div>`;
  charts.quat=mkChart('ch-q',[{l:'W',c:C.blue},{l:'X',c:C.green},{l:'Y',c:C.pink},{l:'Z',c:C.purple}],{min:-1,max:1});
}
function bDist(){
  document.getElementById('cd-dist').innerHTML=`
    <div class="card-lbl">Distance<span class="tag">mm</span></div>
    <div class="big"><span id="vdist">--</span><span class="u">mm</span></div>
    <div class="hint">ToF · Long mode · up to 4 m</div>
    <div class="ch h86"><canvas id="ch-d"></canvas></div>`;
  charts.dist=mkChart('ch-d',[{l:'mm',c:C.orange}]);
}
function bVel(){
  document.getElementById('cd-vel').innerHTML=`
    <div class="card-lbl">Velocity (filtered)<span class="tag">m/s</span></div>
    <div class="big"><span id="vvel">--</span><span class="u">m/s</span></div>
    <div class="hint">Exponential moving average · noise filtered</div>
    <div class="ch h86"><canvas id="ch-v"></canvas></div>`;
  charts.vel=mkChart('ch-v',[{l:'m/s',c:C.teal}],{min:-2,max:2});
}
function bLux(){
  document.getElementById('cd-lux').innerHTML=`
    <div class="card-lbl">Light Intensity<span class="tag">lux</span></div>
    <div class="big"><span id="vlux">--</span><span class="u">lux</span></div>
    <div class="row c3">
      <div class="cell"><div class="cl">Category</div><div class="cv ac" id="vlcat">--</div></div>
      <div class="cell"><div class="cl">Range</div><div class="cv or" id="vlrng">--</div></div>
      <div class="cell"><div class="cl">Log %</div><div class="cv gr" id="vlpct">--</div></div>
    </div>
    <div class="ch h70"><canvas id="ch-l"></canvas></div>`;
  charts.lux=mkChart('ch-l',[{l:'lux',c:C.amber}]);
}

/* ── helpers ── */
function luxCat(v){
  if(v<1)    return['Dark','< 1'];
  if(v<50)   return['Dim','1–50'];
  if(v<300)  return['Indoor','50–300'];
  if(v<1000) return['Bright','300–1k'];
  if(v<10000)return['Sunlit','1k–10k'];
  return['Direct ☀','>10k'];
}
function feel(hi){
  if(hi<10)return'🥶 Cold';
  if(hi<20)return'Comfortable';
  if(hi<26)return'Warm';
  if(hi<32)return'🥵 Hot';
  return'🔥 Danger';
}
function quatCSS(w,x,y,z){
  const sp=2*(w*y-z*x);
  const p=Math.abs(sp)>=1?Math.sign(sp)*90:Math.asin(sp)*180/Math.PI;
  const r=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))*180/Math.PI;
  const ya=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))*180/Math.PI;
  return`rotateX(${p.toFixed(1)}deg) rotateY(${ya.toFixed(1)}deg) rotateZ(${r.toFixed(1)}deg)`;
}
function chip(id,on){
  const e=document.getElementById('chip-'+id);
  on?e.classList.add('on'):e.classList.remove('on');
}

/* ── poll ── */
async function poll(){
  try{
    const r=await fetch('/data'),d=await r.json();
    document.getElementById('ts').textContent=
      new Date().toLocaleTimeString([],{hour:'2-digit',minute:'2-digit',second:'2-digit'});

    chip('bme',d.bme); chip('bno',d.bno); chip('tof',d.tof); chip('tsl',d.tsl);

    if(d.bme){
      if(!init.t){bTemp();init.t=1;} if(!init.h){bHum();init.h=1;}
      if(!init.p){bPres();init.p=1;} if(!init.g){bGas();init.g=1;}
      $v('vt',d.temp.toFixed(1)); $v('vh',d.hum.toFixed(1));
      $v('vdew',d.dew.toFixed(1)+'°'); $v('vhi',d.heatIdx.toFixed(1)+'°');
      $v('vfeel',feel(d.heatIdx));
      $v('vp',d.pres.toFixed(1)); $v('valt',d.alt.toFixed(0)+' m');
      $v('vg',d.gas.toFixed(1));
      push(charts.temp,d.temp);
      push(charts.hum,d.hum,d.dew);
      push(charts.pres,d.pres,d.alt);
      push(charts.gas,d.gas);
    }

    if(d.bno){
      if(!init.e){bEuler();init.e=1;} if(!init.a){bAccel();init.a=1;}
      if(!init.gy){bGyro();init.gy=1;} if(!init.m){bMag();init.m=1;}
      if(!init.q){bQuat();init.q=1;}
      $v('vroll',d.roll.toFixed(1)+'°'); $v('vpitch',d.pitch.toFixed(1)+'°'); $v('vyaw',d.yaw.toFixed(1)+'°');
      $v('vax',d.ax.toFixed(3)); $v('vay',d.ay.toFixed(3)); $v('vaz',d.az.toFixed(3));
      $v('vgx',d.gx.toFixed(2)); $v('vgy',d.gy.toFixed(2)); $v('vgz',d.gz.toFixed(2));
      $v('vmx',d.mx.toFixed(2)); $v('vmy',d.my.toFixed(2)); $v('vmz',d.mz.toFixed(2));
      $v('vqw',d.qw.toFixed(4)); $v('vqx',d.qx.toFixed(4));
      $v('vqy',d.qy.toFixed(4)); $v('vqz',d.qz.toFixed(4));
      const cb=document.getElementById('cube3d');
      if(cb) cb.style.transform=quatCSS(d.qw,d.qx,d.qy,d.qz);
      push(charts.euler,d.roll,d.pitch,d.yaw);
      push(charts.accel,d.ax,d.ay,d.az);
      push(charts.gyro,d.gx,d.gy,d.gz);
      push(charts.mag,d.mx,d.my,d.mz);
      push(charts.quat,d.qw,d.qx,d.qy,d.qz);
    }

    if(d.tof){
      if(!init.d){bDist();init.d=1;} if(!init.v){bVel();init.v=1;}
      $v('vdist',d.dist); $v('vvel',d.vel.toFixed(3));
      push(charts.dist,d.dist); push(charts.vel,d.vel);
    }

    if(d.tsl){
      if(!init.l){bLux();init.l=1;}
      const[cat,rng]=luxCat(d.lux);
      const pct=Math.min(100,(Math.log10(Math.max(1,d.lux))/5*100)).toFixed(0);
      $v('vlux',d.lux.toFixed(0));
      $v('vlcat',cat); $v('vlrng',rng); $v('vlpct',pct+'%');
      push(charts.lux,d.lux);
    }

  }catch(e){
    document.getElementById('ts').textContent='Reconnecting…';
  }
  setTimeout(poll,50);
}
poll();
</script>
</body>
</html>
)JS"));
}

void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n★ MagicCube v2.1 ★");
  Serial.println("→ Velocity stabilized with EMA filter (m/s)");
  Serial.println("→ Accelerometer in m/s² (BNO055 native)");

  pinMode(XSHUT_PIN,OUTPUT); digitalWrite(XSHUT_PIN,HIGH); delay(10);
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000);

  if(bme.begin(0x76,&Wire)){
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320,150);
    bme_ok=true; Serial.println("[OK] BME688");
  } else Serial.println("[--] BME688 not found");

  if(bno.begin()){
    bno.setExtCrystalUse(true);
    bno_ok=true; Serial.println("[OK] BNO055 (Accel = m/s²)");
  } else Serial.println("[--] BNO055 not found");

  tof.setTimeout(500);
  if(tof.init()){
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(20000);
    tof.startContinuous(20);
    tof_ok=true; Serial.println("[OK] VL53L1X");
  } else Serial.println("[--] VL53L1X not found");

  if(tsl.begin()){
    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    tsl_ok=true; Serial.println("[OK] TSL2561");
  } else Serial.println("[--] TSL2561 not found");

  WiFi.mode(WIFI_AP); WiFi.softAP("MagicCube");
  Serial.print("\nHotspot : MagicCube\nDashboard: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/",handleRoot);
  server.on("/data",handleData);
  server.begin();
  Serial.println("Web server ready.\n");
}

void loop(){ 
  server.handleClient(); 
}