# Jog operation with keyboard 
#
# Copyright (c) 2023 MODECO
# Released under the MIT license.
# see https://opensource.org/licenses/MIT

# [NOTE]
# (need to start pigpio daemon before running this script, )
# sudo pigpiod
# 
# (to install pigpio, )
# sudo apt-get install pigpio

import time
import utilities
import pigpio


# pigpioのhardware_PWMでは1000000が100% dutyを表す。
# duty比[%]をpigpioへ渡す整数値に変換する。
def duty100(rate):
    # 安全のため、入力されたduty比[%]をこの範囲に制限する。
    PWM_SAFE_MIN = 7.50 
    PWM_SAFE_MAX = 13.00
    if rate > PWM_SAFE_MAX:
        rate = PWM_SAFE_MAX
    if rate < PWM_SAFE_MIN:
        rate = PWM_SAFE_MIN
    return int(rate * 10000)

# 初期化
print("PWM Jog control...")

# 校正パラメータ。実車に合わせて調整する値。

Delta_Jog = 0.145 # キー1回あたりの調整量

#PWM_Hz = 60 
#PWM_NeutralSpd = 8.695  #@60Hz 
#PWM_NeutralStr = 8.695  #@60Hz

# PWM周波数[Hz]。
# RCサーボやESCへ送るPWM信号の繰り返し周波数。
# この設定では1周期が約14.3 msになる。
PWM_Hz = 70

# ステアリング中立位置のduty比[%]。
# 70 Hzでは1周期が約14.3 msなので、10.88%は約1.55 msのパルス幅に相当する。
# 車両がまっすぐ向く値へ実車で校正する。
PWM_NeutralStr = 10.88 #@70Hz

# ESCの速度中立位置のduty比[%]。
# 70 Hzでは10.55%が約1.51 msのパルス幅に相当する。
# モーターが前進も後退もしない値へ実車で校正する。
PWM_NeutralSpd = 10.55 #@70Hz
PWM_CruiseSpd = 10.0 + 1.4 #@70Hz # 巡航速度のduty比[%]
RUN_SECONDS = 10.0  # 最大実行時間[s]

# 現在の指令値。起動時は速度・ステアリングとも中立から始める。
spd_ref = PWM_NeutralSpd
str_ref = PWM_NeutralStr

# PWM出力ピン。Raspberry Piのhardware PWMは12/13または18/19で使用できる。
gppin_acc = 12
gppin_str = 13

# pigpioへ接続し、PWM出力ピンを初期化する。
pi = pigpio.pi()
pi.set_mode(gppin_acc, pigpio.OUTPUT)
pi.set_mode(gppin_str, pigpio.OUTPUT)


# 起動直後は一度中立付近の信号を出して、ESCとサーボを安定させる。
pi.hardware_PWM(gppin_acc, PWM_Hz, duty100(10.5))
pi.hardware_PWM(gppin_str, PWM_Hz, duty100(10.5))
time.sleep(1)

def write_help():
    print("Enter: stop, w&d:speed, a&d:steering, auto stop after {:.1f} s".format(RUN_SECONDS))

write_help()

try:                        # try:の部分にループ処理を書く
    i = 0
    start_time = time.monotonic()
    while True:
        # ループ回数を数える。
        i = i + 1

        # 指定時間を超えたら中立停止処理へ進む。
        if time.monotonic() - start_time >= RUN_SECONDS:
            print("time limit reached")
            break
        
        # キー入力を確認する。入力がない場合は0が返る。
        key = utilities.getkey()
        if key == 10:
            break
        if key == ord('w'):
            # 速度指令を小さくする。
            spd_ref = spd_ref - Delta_Jog * 2
        if key == ord('s'):
            # 速度指令を大きくする。
            spd_ref = spd_ref + Delta_Jog * 2
        if key == ord('a'):
            # ステアリングを左方向へ動かす。
            str_ref = str_ref - Delta_Jog * 4
        if key == ord('d'):
            # ステアリングを右方向へ動かす。
            str_ref = str_ref + Delta_Jog * 4 
        if key == ord('n'):
            # 速度とステアリングを中立へ戻す。
            str_ref = PWM_NeutralStr
            spd_ref = PWM_NeutralSpd
        # 現在の速度指令をESCへ出力する。
        pi.hardware_PWM(gppin_acc, PWM_Hz, duty100(spd_ref))
        # 現在のステアリング指令をサーボへ出力する。
        pi.hardware_PWM(gppin_str, PWM_Hz, duty100(str_ref))
        
        # 現在の指令値を定期的に表示する。
        if i % 20 == 0:
            print("str,spd=",'{:.4g}'.format(str_ref), '{:.4g}'.format(spd_ref))
        if i % 200 == 0:
            write_help()
            
        # 制御周期を約10 msにする。
        time.sleep(0.01)
        
        
except KeyboardInterrupt:   # exceptに例外処理を書く
    print('stop!')
    # Ctrl-C時は速度を中立へ戻す。
    pi.hardware_PWM(gppin_acc, PWM_Hz, duty100(PWM_NeutralSpd))
    # Ctrl-C時はステアリングを中立へ戻す。
    pi.hardware_PWM(gppin_str, PWM_Hz, duty100(PWM_NeutralStr))
    #pi.stop()

# 通常終了時も速度を中立へ戻す。
pi.hardware_PWM(gppin_acc, PWM_Hz, duty100(PWM_NeutralSpd))
# 通常終了時もステアリングを中立へ戻す。
pi.hardware_PWM(gppin_str, PWM_Hz, duty100(PWM_NeutralStr))
#pi.stop()
print("finish.")
