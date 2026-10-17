# 静止调回中与方向检查（20261017.3）

本入口处理“steer_command=0时前轮仍明显向右”的问题。先找实际中位，再确认正负方向，
暂不翻转入库命令，也不改变原VV公式、PWM2通道、3040000ns周期及原硬件初始化调用顺序。
不需要量角器，只观察前轮是否与车身纵向基本平行。

## 文件与作用范围

- 回中安装偏移只改板端本目录的`image_params.txt`：`steer_offset_deg=95`这一行。
- 原读取范围为-150..150，支持小数；这是软件范围，不是机构已验证范围。
- `config/parking_tuning.vv.ini`的各阶段PWM、时间、角度全部保留；静止模式不读取这份停车配置。
- 调好的偏移会影响正常巡线和所有入库阶段的命令0。原公式继续由Steer::setAngle唯一计算。
- 软件正负只代表命令增减；前轮实际左右方向需要本车观察确认。

## 编译与保留板端已调参数的更新

在Linux虚拟机的新源码目录：

```sh
cd /mnt/hgfs/share/vv15_auto_parking_20261017_3
bash build.sh
```

脚本已补入本机确认的`$HOME/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4`，
也支持显式设置`OpenCV_DIR`。编译后将新`vv15_auto_parking_board.zip`上传到板端原目录。
先退出所有小车程序、关闭舵机开关，再执行下面更新；排除两份参数文件，避免覆盖你的实测值：

```sh
cd /home/root/gy2/vv15_auto_parking
unzip -o vv15_auto_parking_board.zip -x image_params.txt config/parking_tuning.vv.ini
sha256sum -c AUTO_PARKING_PROGRAM_SHA256SUMS
chmod +x vv_auto_parking
```

`AUTO_PARKING_PROGRAM_SHA256SUMS`校验程序/脚本/文档/图片，不要求修改过的参数与模板相同。
`AUTO_PARKING_SHA256SUMS`校验整个出厂包，适用于全新完整解压；已调参数的更新使用前者。
当前源码包不含新龙芯可执行文件，需要重新编译，不要拿上一版二进制配新脚本。

## 操作：一个终端运行，另一个终端改文件

1. 关闭舵机开关，执行：

   ```sh
   sh /home/root/gy2/vv15_auto_parking/run_servo_center.sh
   ```

2. 看到`SERVO_CENTER_READY`后，第一次保持舵机关着，Ctrl+C退出；再次运行同一命令。
   每次都沿用原全局硬件初始化顺序。电机、编码器、相机对象、舵机都会初始化，但不启动
   图像线程或电机控制线程，不巡线、不录像、不执行任何入库阶段。
3. 第二次看到提示后，打开舵机开关。保持电机动力关闭进行观察；程序本身只向两路电机写0。
   点击**正在运行静止模式的终端**，按一次Enter，写入“现有偏移+命令0”。
   控制台会显示`APPLIED offset=... command=0 duty_readback_ns=...`。
4. 在MobaXterm另一个终端备份、修改同目录`image_params.txt`，也可用左侧文件窗口编辑保存。
   例如当前值确实是95时，先尝试减小一档到90（这只是观察用试值，不是已确定中位）：

   ```sh
   cd /home/root/gy2/vv15_auto_parking
   cp -p image_params.txt "image_params.before_center_$(date +%Y%m%d_%H%M%S).txt"
   grep '^steer_offset_deg=' image_params.txt
   sed -i 's/^[[:space:]]*steer_offset_deg[[:space:]]*=.*/steer_offset_deg=90/' image_params.txt
   ```

5. **回到运行静止模式的终端，再按Enter应用命令0。**文件保存本身不会执行转向。
   如果减小偏移使前轮更接近直行，就继续每次减5；若更偏，则恢复上一值，往相反方向调。
   接近直行后每次改1，必要时用小数细调。不要一次跳到软件极值；若机构顶住就停止该方向调整。
6. 找到命令0基本回正后，输入`-`并Enter，观察小幅负命令；按Enter回到0；
   输入`+`并Enter，观察小幅正命令；再Enter回到0。把“负命令向哪边、正命令向哪边”记录下来。
7. 结束前关闭舵机开关，然后`Q`加Enter或Ctrl+C。退出不会转入巡线、不会执行整套入库。
   不会在退出时额外强制舵机回中；退出前可自行Enter回到0。

修改文件后，终端仅在数值改变时提示一次待应用；保持静止时不会连续刷相同提示。
文件缺键、重复键、越界、NaN等无效值会拒绝应用，电机继续为0。
电机零值写入或舵机写入失败时退出该模式，输出具体错误。

## 按键与数据

| 输入（字符后按Enter） | 动作 |
|---|---|
| 空行Enter 或 0 | 读取当前文件的偏移，应用命令0，观察回中 |
| - | 读取当前偏移，应用固定命令-5，观察实际方向 |
| + | 读取当前偏移，应用固定命令+5，观察实际方向 |
| Q | 结束静止模式；先关舵机 |
| Ctrl+C | 结束静止模式；先关舵机 |

不接受任意角度数字。没有后台自动扫角；无按键授权时不额外改变舵机。
初始化本身的原有舵机写入仍保留，所以首次启动仍关闭舵机开关。
`duty_readback_ns`是sysfs回读，不是实测轮角或示波器测量。

原公式相当于：

```text
duty_ns = 1520000 + (steer_command + steer_offset_deg) / 90 * 5 * (3040000 / 100)
```

偏移95、命令0约1680444ns；偏移90、命令0约1672000ns。
1.52ms只是原公式基准，不能直接认定为这辆车的机械中位。
本入口没有自动替你修改偏移，最终值以你实际观察并保存在image_params.txt中的值为准。

## 恢复入库

确认中位和实际正负方向后，先退出静止模式，再按原流程运行：

```sh
sh /home/root/gy2/vv15_auto_parking/start_parking.sh
```

启动脚本已内置板端已确认的OpenCV目录，每次新终端无需手工export。
如果回中正确但负命令仍向右，先反馈静止方向检查结果，再协调停车转向映射；
不要只把stage_2负数改正数绕过方向约束，也不要据车尾轨迹判断前轮左右。
