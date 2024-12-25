from machine import Pin, I2C, ADC, PWM
import time
import math

# กำหนดพอร์ต I2C (SDA=23, SCL=22)
i2c = I2C(0, scl=Pin(22), sda=Pin(23))

button_1 = Pin(16, Pin.IN, Pin.PULL_UP)  # ปุ่มที่ต่อกับ Pin 16
button_2 = Pin(17, Pin.IN, Pin.PULL_UP)  # ปุ่มที่ต่อกับ Pin 17

potentiometer = ADC(Pin(34))
potentiometer.atten(ADC.ATTN_11DB)  # เพิ่มช่วงแรงดันไฟฟ้าจาก 0-3.3V
potentiometer.width(ADC.WIDTH_12BIT)  # กำหนดความละเอียดเป็น 12 บิต (ค่า 0-4095)

servo_left = PWM(Pin(12), freq=50)  # Servo ตัวที่ 1 ใช้ Pin 12
servo_right = PWM(Pin(14), freq=50)  # Servo ตัวที่ 2 ใช้ Pin 14
    
def read_potentiometer():
    value = potentiometer.read()  # อ่านค่า ADC (0-4095)
    return value

def read_buttons():
    state_1 = not button_1.value()  # อ่านค่า Pin 16 (active-low: 0 = กด, 1 = ปล่อย)
    state_2 = not button_2.value()  # อ่านค่า Pin 17 (active-low: 0 = กด, 1 = ปล่อย)
    return state_1, state_2

# สแกนอุปกรณ์ I2C
devices = i2c.scan()
if not devices:
    print("No I2C devices found!")
else:
    print("I2C devices found at:", [hex(dev) for dev in devices])

# ตรวจสอบว่าที่อยู่ถูกต้องหรือไม่
MMA8452_ADDR = 0x1C  # เปลี่ยนตามผลลัพธ์จาก i2c.scan()

# ฟังก์ชันเริ่มต้นเซ็นเซอร์
def init_mma8452():
    i2c.writeto_mem(MMA8452_ADDR, 0x2A, b'\x01')  # Active mode

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

# ฟังก์ชันอ่านค่าแกน X, Y, Z
def read_mma8452_mapped():
    data = i2c.readfrom_mem(MMA8452_ADDR, 0x01, 6)
    x = (data[0] << 4 | (data[1] >> 4))
    y = (data[2] << 4 | (data[3] >> 4))
    z = (data[4] << 4 | (data[5] >> 4))
    if x > 2047: x -= 4096
    if y > 2047: y -= 4096
    if z > 2047: z -= 4096

    # Map ค่าให้มาอยู่ในช่วง 0-100 (ช่วงเซ็นเซอร์: -2048 ถึง 2047)
    x_mapped = map_value(x, -2048, 2047, 0, 1000)
    y_mapped = map_value(y, -2048, 2047, 0, 1000)
    z_mapped = map_value(z, -2048, 2047, 0, 1000)

    return x_mapped, y_mapped, z_mapped

# ฟังก์ชันปรับมุม Servo (0-180 องศา)
def set_angle(servo, angle):
    # Map มุม (0-180) ไปยัง Duty Cycle ที่เหมาะสม
    min_duty = 1000  # Duty สำหรับ 0 องศา (1 ms)
    max_duty = 9000  # Duty สำหรับ 180 องศา (2 ms)
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo.duty_u16(duty)


servo_update_gap=1
left_update=0
right_update=0

def servo_controller(left, right):
    global left_update, right_update
    if isinstance(left, int) and left_update != left and math.fabs(left_update-left) >= servo_update_gap:
        left_update=left
        set_angle(servo_left, left)
        
    if isinstance(right, int) and right_update != right and math.fabs(right_update-right) >= servo_update_gap:
        right_update=right
        set_angle(servo_right, right)

# เริ่มต้นเซ็นเซอร์
if MMA8452_ADDR in devices:
    init_mma8452()
    while True:
        button_1_state, button_2_state = read_buttons()
        if button_1_state:
            if button_2_state:
                x, y, z = read_mma8452_mapped()
                # print("get: {} {} {}".format(x, y, z))
                
                if x<490:
                    angle_auto=map_value(x,246,490,45,90)
                    servo_controller(angle_auto,90)
                    print("Auto left: {}".format(angle_auto))
                elif x>510:
                    angle_auto=map_value(x,510,745,90,135)
                    servo_controller(90,angle_auto)
                    print("Auto right: {}".format(angle_auto))
                else:
                    servo_controller(90,90)
                    print("Auto balance: 90")
            else:
                adc_value = read_potentiometer()
                angle_manual=map_value(adc_value,0,4095,45,135)
                servo_controller(angle_manual,angle_manual)
                print("Manual: {}".format(angle_manual))
        else:
            servo_controller(90,90)
            print("OFF: Set 90")
        time.sleep(0.01)
else:
    print("MMA8452 not found at address 0x{:02X}".format(MMA8452_ADDR))