```
F405_config.bin*    --->F405配置文件，烧写位置为0x08004000
gateway1/           --->网关的APP，bin文件的烧写位置为0x08020000
iap_gw/             --->网关的bootload，bin文件的烧写位置为0x08000000
endpoint1/          --->终端的工程目录
f103_cofig.bin*     --->F103的配置文件，烧写位置为0x0800F800
f103_key.bin        --->F103的key文件，烧写位置为0x0800FC00
iap_server/         --->F103的bootload，bin文件烧写位置为0x08000000
server/             --->F103的APP，bin文件的烧写位置为0x08006400
README.md* 			--->说明文件
firmware_generate.py -->网关和server出厂固件的生成
```
###firmware_generate.py的使用说明
***前提：***    
需要提前将`gw_iap.bin`，`iap_server.bin`，`STM32F103.bin`， `STM32F405.bin`编译出来。    

***运行环境：python2.0***    
window,linux,mac安装python2.0就可以运行

```
命令的执行：
python firmware_generate.py 1.0 1.0 1000 1b 1c
./firmware_generate.py 1.0 1.0 1000 1b 1c

参数说明：
./firmware_generate.py F405版本号 F103版本号 server地址 server_channel_1 server_channel_2
F405版本号 : 格式为x.x
F103版本号 : 格式为x.x
server地址 : 16进制数，例如F000，取值范围是（0～FFFF）
server_channel_1 : 16进制数，例如1b，取值范围是（0～1F）
server_channel_2 : 16进制数，例如1b，取值范围是（0～1F）

```
```
举例：
python firmware_generate.py 1.0 1.0 1000 1b 1c
['firmware_generate.py', '1.0', '1.0', '1000', '1b', '1c']
开始检查需要的文件是否都存在......
文件检查完毕：OK
开始生成F405 firmware......
F405 firmware 生成完毕 OK
开始生成F103 firmware......
F103 firmware 生成完毕: OK
生成的文件如下：
f405_firmware.bin
f103_firmware.bin
liyongsheng-2:bin liyongsheng$ 

```
 
