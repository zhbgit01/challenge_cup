// ==================== 1. 通信库与参数 ====================
// Wire 是 Arduino 自带的 I²C 通信库，UNO 使用 A4(SDA)、A5(SCL)。
#include <Wire.h>
// uint8_t 为8位无符号整数；uint32_t 为32位无符号整数。
// 全局变量未显式赋值时初始为0；address 用来保存找到的传感器地址。
uint8_t address;
const uint32_t SAMPLE_PERIOD_US = 1000000; // 采样间隔：1000000微秒 = 1秒
// 下一次应该读取传感器的时间，使用 micros() 的微秒计时。
uint32_t nextSample;
const uint8_t ALARM_PIN = 8; // 报警模块的 IN 接 UNO D8
const uint8_t ALARM_ACTIVE_LEVEL = LOW; // 实物为低电平触发：LOW响亮，HIGH关闭
const float Z_THRESHOLD = 8.0f; // Z轴阈值，单位m/s²，包含重力；f表示float常量
const uint32_t BLINK_PERIOD_MS = 250; // 每250毫秒切换一次：亮响250ms、停250ms
bool alarming = false; // 是否处于超标报警状态，保持到下一次采样判断
bool alarmOn = false; // 当前闪烁阶段是否正在亮响，与“是否超标”分开记录
uint32_t lastBlink; // 上一次切换声光输出的时间，单位毫秒

// ==================== 2. 控制一次声光输出 ====================
// on=true 开启声光，on=false 关闭，同时控制 UNO 板载指示灯。
void setAlarmOutput(bool on) {
  alarmOn = on;
  // 条件 ? 值A : 值B 是三目运算：条件成立选A，否则选B。
  // 开启时输出有效电平，关闭时输出相反电平。
  digitalWrite(ALARM_PIN, on ? ALARM_ACTIVE_LEVEL : (ALARM_ACTIVE_LEVEL == HIGH ? LOW : HIGH));
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

// ==================== 3. 更新是否超标 ====================
// 仅在状态变化时重置闪烁计时，持续超标时让原来的闪烁节奏继续。
void updateAlarm(bool enabled) {
  if (enabled != alarming) {
    alarming = enabled;
    lastBlink = millis();
    setAlarmOutput(enabled); // 刚超标立即亮响；恢复正常立即关闭
  }
}

// ==================== 4. 上电初始化（只执行一次） ====================
void setup() {
  // 先预设关闭电平，再切换为输出，减少初始化时的误触发。
  digitalWrite(ALARM_PIN, ALARM_ACTIVE_LEVEL == HIGH ? LOW : HIGH);
  pinMode(ALARM_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  setAlarmOutput(false);
  Serial.begin(115200); // 与电脑通信，串口绘图器也要选择115200
  // 初始化I²C后关闭UNO内部5V上拉，依赖传感器模块已有的外部上拉。
  // 此处LOW用于关闭输入引脚的内部上拉，并非将SDA/SCL设为普通输出。
  Wire.begin();
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  Wire.setClock(100000); // I²C时钟100kHz，不是加速度采样频率
  Wire.setWireTimeout(25000, true); // 总线等待超时25ms，超时后重置通信外设
  delay(100); // 给传感器上电稳定的时间
  // SDO引脚状态决定地址，依次尝试两种地址。
  const uint8_t candidates[] = {0x53, 0x1D};
  for (uint8_t candidate : candidates) {
    address = candidate;
    uint8_t id;
    // 0x00为设备编号寄存器；ADXL345的编号应为0xE5。
    // &id 表示id的内存地址，让读取函数将结果写入id。
    if (readRegisters(0x00, &id, 1) && id == 0xE5) break;
    address = 0;
  }
  if (!address) { // 两个地址都未识别成功：停止采样并每2秒报告错误
    while (true) {
      // F()把固定文字放在程序存储器中，节省UNO的运行内存。
      Serial.println(F("ERROR: ADXL345 not found. Check wiring and CS/SDO."));
      delay(2000);
    }
  }
  // 向寄存器写入配置：
  // 0x31(DATA_FORMAT) ← 0x09：全分辨率、±4g量程。
  // 0x2C(BW_RATE)     ← 0x0A：传感器内部以100Hz更新读数。
  // 0x2D(POWER_CTL)   ← 0x08：进入测量模式。
  // UNO仍然每秒只取一次当前读数，不计算一秒平均值。
  // || 表示“或”，任一写入失败就停止并报告配置错误。
  if (!writeRegister(0x31, 0x09) || !writeRegister(0x2C, 0x0A) || !writeRegister(0x2D, 0x08)) {
    while (true) {
      Serial.println(F("ERROR: configuration failed."));
      delay(2000);
    }
  }
  delay(30); // 等待测量值更新
  nextSample = micros(); // 进入loop后立即进行第一次采样
}

// ==================== 5. 主循环（反复执行） ====================
void loop() {
  // 5.1 报警时独立计时闪烁，不用delay阻塞，所以采样等待期间也能闪烁。
  if (alarming && (uint32_t)(millis() - lastBlink) >= BLINK_PERIOD_MS) {
    lastBlink = millis();
    setAlarmOutput(!alarmOn);
  }
  // 5.2 检查是否到达采样时间；未到就结束本次loop，马上开始下一轮。
  // 用时间差比较，以兼容micros()/millis()计数回绕。
  uint32_t now = micros();
  if ((int32_t)(now - nextSample) < 0) return;
  nextSample += SAMPLE_PERIOD_US;
  // 如果处理耗时过长错过了下一次采样，则重新安排，避免连续补采。
  if ((int32_t)(now - nextSample) >= 0) nextSample = now + SAMPLE_PERIOD_US;

  // 5.3 从0x32开始连续读取6字节：X低/高、Y低/高、Z低/高。
  uint8_t data[6];
  if (!readRegisters(0x32, data, 6)) {
    updateAlarm(false); // 读取失败时关闭报警，并报告故障；不代表测量正常
    Serial.println(F("ERROR: sensor read failed."));
    return;
  }
  // 5.4 逐轴拼接原始数据、换算单位，并输出给电脑绘图。
  const char labels[] = {'X', 'Y', 'Z'};
  float z = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    // 高字节左移8位，再用按位或 | 合并低字节。
    // 转成有符号16位整数int16_t后，才能正确表示负加速度。
    int16_t raw = (int16_t)((uint16_t)data[axis*2] | ((uint16_t)data[axis*2+1] << 8));
    Serial.print(labels[axis]); Serial.print(':');
    // 全分辨率下每个计数名义约为0.0039g；乘9.80665换成m/s²。
    // 未去除重力、未做零偏校准，静止时某轴也可能接近±9.8。
    float acceleration = raw * (0.0039f * 9.80665f);
    if (axis == 2) z = acceleration; // 第三个轴为Z，保存供报警判断
    Serial.print(acceleration, 3); // 保留小数点后3位
    // 制表符分隔各轴，换行结束一组，例如 X:0.100 Y:0.200 Z:9.800。
    if (axis < 2) Serial.print('\t'); else Serial.println();
  }
  // 5.5 严格超过±8才报警：等于8或-8不触发。
  // 本次判断一直保持到下一次读取；两次采样之间的短暂超标可能漏掉。
  updateAlarm(z > Z_THRESHOLD || z < -Z_THRESHOLD);
}

// ==================== 6. I²C连续读取函数 ====================
// reg：起始寄存器；data：接收数组；count：字节数；返回true表示读取成功。
bool readRegisters(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg); // 告诉传感器从哪个寄存器开始读取
  if (Wire.endTransmission(false) != 0) return false; // 保持总线，准备重复起始读取
  if (Wire.requestFrom(address, count) != count) return false; // 检查收到的字节数
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

// ==================== 7. I²C单寄存器写入函数 ====================
// 将value写到reg寄存器，供初始化配置传感器使用。
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0; // 返回码0表示发送成功
}
