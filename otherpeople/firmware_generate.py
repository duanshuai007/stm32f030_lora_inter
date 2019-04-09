#!/usr/bin/python
# -*- coding: UTF-8 -*-
import os
import getopt
from struct import Struct
import sys
import hashlib

print(sys.argv)

def help_info():
  print('\r\n')
  print(sys.argv[0] + ' 网关ID' + ' F405版本号' + ' F103版本号' + ' channel')
  print('网关ID : 最长32字节')
  print('F405版本号 : 格式为x.x')
  print('F103版本号 : 格式为x.x')  
  print('channel : 16进制数，例如1b，取值范围是（30～31）') 
  print('\r\n') 


#=========================================
#校验脚本执行时的参数是否正确
#=========================================
if not len(sys.argv)==8:
  print('参数个数错误，正确的格式为:')
  help_info()
  exit(0)
gw_id = sys.argv[1]
if len(gw_id) >= 32:
  print('网关ID参数长度太长（需要小于等于32）')
  help_info()
  exit(0)

versionF405 = sys.argv[2]
if len(versionF405) > 3:
  print('F405versin参数error，正确的格式为:')
  help_info()
  exit(0)

versionF103 = sys.argv[3]
if len(versionF103) > 3:
  print('F103version参数error，正确的格式为:')
  help_info()
  exit(0)

channel = int(sys.argv[4])

endpointIDStart = int(sys.argv[5])
endPointIDNum = int(sys.argv[6])
increase = int(sys.argv[7])

#先判断f405_firmware.bin文件是否存在，如果存在，先删除
f405path = './f405_'+sys.argv[1]+'_'+sys.argv[2]+'_'+sys.argv[3]+'_'+sys.argv[4]+'_'+sys.argv[5]+'_'+sys.argv[6]+'_'+sys.argv[7]+'.bin';
f103path = './f103_'+sys.argv[1]+'_'+sys.argv[2]+'_'+sys.argv[3]+'_'+sys.argv[4]+'_'+sys.argv[5]+'_'+sys.argv[6]+'_'+sys.argv[7]+'.bin';
print(f405path)
if os.path.exists(f405path):
    os.remove(f405path)

f405allFile = open(f405path, 'ab')

#=========================================
#检查需要的文件是否存在
#=========================================

print('开始检查需要的文件是否都存在......')
if not os.path.exists('./gw_iap.bin'):
    print("gw_iap.bin不存在，请检查gw_iap.bin文件是否拷贝到同级目录！");
    exit(0)

if not os.path.exists('./STM32F405.bin'):
    print("STM32F405.bin不存在，请检查STM32F405.bin文件是否拷贝到同级目录！");
    exit(0)

if not os.path.exists('./iap_server.bin'):
    print("iap_server.bin不存在，请检查iap_server.bin文件是否拷贝到同级目录！");
    exit(0)

if not os.path.exists('./STM32F103.bin'):
    print("STM32F103.bin不存在，请检查STM32F103.bin文件是否拷贝到同级目录！");
    exit(0) 
print('文件检查完毕：OK')
#=========================================
#写入F405的bootload
#=========================================
print('开始生成F405 firmware......')
#写入F405的bootload内容
with open('gw_iap.bin', 'rb') as f:
    image_data = f.read()
f405allFile.write(image_data)

#将F405 bootload分区的剩余空间填充0xff
size = os.path.getsize('./gw_iap.bin')
remain = 16*1024 - size

remain_struct = Struct('<B')
init_remain = remain_struct.pack(0xFF)
for i in range(0, remain):
  f405allFile.write(init_remain)


#=========================================
#写入F405的config文件
#=========================================
#写入uiUpdateVerAddr（0x08040000, 4个字节）
#写入uiRecoveryVerAddr（0x08060000, 4个字节）
#写入uiUpdateVerDataTotalLen（0, 4个字节）
#写入uiRecoveryVerDataLen（根据F405 App的大小进行计算, 4个字节）
#写入ucTryTimes（0，1字节）
#写入ucNeedUpdateFlg（0，1字节）
#写入ucBootSeccessFlg（1，1字节）
#写入ucRev（0，1字节）
#写入uiF103RunRomAddr（0x08080000, 4个字节）
#写入uiF103UpdateRomAddr（0x080A0000, 4个字节）
#写入ucFWVer（根据实际的版本号填写, 4个字节）
#写入ucNewFWVer（0，4个字节）
config_struct = Struct('<4I4B2I4s4B')
F405size = os.path.getsize('./STM32F405.bin')
init_config = config_struct.pack(0x08040000, 0x08060000, 0, F405size, 0, 0 ,1, 0, 0x08080000, 0x080A0000, versionF405, 0,0,0,0)

f405allFile.write(init_config)

#将F405 config分区的剩余空间填充0xff
remain = 16*1024 - config_struct.size

for i in range(0, remain):
  f405allFile.write(init_remain)

#=========================================
#写入mqtt channel name到第2分区（16k）
#=========================================
mqttCh_config_struct = Struct('<32s')
mqttCh_init_config = mqttCh_config_struct.pack(gw_id)

f405allFile.write(mqttCh_init_config)

#将F405 config分区的剩余空间填充0xff
remain = 16*1024 - mqttCh_config_struct.size

for i in range(0, remain):
  f405allFile.write(init_remain)



#=========================================
#写入0xff到第3，4分区（16k+16k+64k）
#=========================================

remain = 16*1024 + 64*1024
for i in range(0, remain):
  f405allFile.write(init_remain)

#=========================================
#写入STM32F405.bin到第5分区（128k，固件之外填充0xff）
#=========================================

#写入F405的STM32F405.bin内容
with open('STM32F405.bin', 'rb') as f:
    image_data = f.read()
f405allFile.write(image_data)

#将第5分区的剩余空间填充0xff
size = os.path.getsize('./STM32F405.bin')
remain = 128*1024 - size

for i in range(0, remain):
  f405allFile.write(init_remain)

#=========================================
#写入0xff到第6分区（128k）
#=========================================
remain = 128*1024

for i in range(0, remain):
  f405allFile.write(init_remain)

#=========================================
#写入STM32F405.bin到第7分区（128k，固件之外填充0xff）
#=========================================
#写入F405的bootload内容
with open('STM32F405.bin', 'rb') as f:
    image_data = f.read()
f405allFile.write(image_data)

#将该分区的剩余空间填充0xff
size = os.path.getsize('./STM32F405.bin')
remain = 128*1024 - size

for i in range(0, remain):
  f405allFile.write(init_remain)

#=========================================
#写入STM32F103.bin到第8分区（128k，固件之外填充0xff）
#=========================================
size = os.path.getsize('./STM32F103.bin')

#计算F103固件的Md5，填充到head
f103File = open('STM32F103.bin', 'rb')
f103BinData = f103File.read();
f103md5 = hashlib.md5(f103BinData).hexdigest()

#写入F103固件的head
head_struct = Struct('<32s4sI')
init_head = head_struct.pack(f103md5, versionF103, size)
f405allFile.write(init_head)

#写入STM32F103.bin
f405allFile.write(f103BinData)

f103File.close

remain = 128*1024 - size - head_struct.size
#该分区剩余空间填写0xff
for i in range(0, remain):
  f405allFile.write(init_remain)

#=========================================
#写入0xff到第9，10，11分区
#=========================================
remain = 128*1024 + 128*1024 + 128*1024

for i in range(0, remain):
  f405allFile.write(init_remain)

f405allFile.close

print('F405 firmware 生成完毕 OK')
###################################################
###################################################
#
#     生成F103固件
#
###################################################
###################################################
print('开始生成F103 firmware......')
#先判断f103_firmware.bin文件是否存在，如果存在，先删除
if os.path.exists(f103path):
    os.remove(f103path)

f103allFile = open(f103path, 'ab')

#=========================================
#写入F103的bootload
#=========================================
#写入F103的bootload内容
with open('iap_server.bin', 'rb') as f:
    image_data = f.read()
f103allFile.write(image_data)

#将F103 bootload分区的剩余空间填充0xff
size = os.path.getsize('./iap_server.bin')
remain = 25*1024 - size

for i in range(0, remain):
  f103allFile.write(init_remain)

#=========================================
#写入F103的固件
#=========================================
#写入F103的bootload内容
with open('STM32F103.bin', 'rb') as f:
    image_data = f.read()
f103allFile.write(image_data)

#将F103 bootload分区的剩余空间填充0xff
size = os.path.getsize('./STM32F103.bin')
remain = 37*1024 - size

for i in range(0, remain):
  f103allFile.write(init_remain)

#=========================================
#写入F103的config
#=========================================
#写入ucBootType（0，1字节）
#写入ucTryTimes（0，1字节）
#写入ucBootSeccessFlg（1，1字节）
#写入ucRev（0，1字节）
#写入ucFWVer（根据实际的版本号填写, 4个字节）
#写入ucNewFWVer（0，4个字节）
f103_config_struct = Struct('<4B4s4B')
f103_init_config = f103_config_struct.pack(0, 0 , 1, 0, versionF103, 0,0,0,0)

f103allFile.write(f103_init_config)

#将F405 config分区的剩余空间填充0xff
remain = 1*1024 - f103_config_struct.size

for i in range(0, remain):
  f103allFile.write(init_remain)

#=========================================
#写入F103的key
#=========================================
#写入server add（2字节）
#写入channel（根据用户指定, 2个字节）
f103_config_struct = Struct('<hh')
f103_init_config = f103_config_struct.pack(4096, channel)

f103allFile.write(f103_init_config)

#将key和endpointID之间的剩余空间填充0xff
remain = 48 - f103_config_struct.size

for i in range(0, remain):
  f103allFile.write(init_remain)

#写入endpointId的值
epID_struct = Struct('<h')
if increase>=1:
  for i in range(endpointIDStart, endpointIDStart+endPointIDNum):  
    epID = epID_struct.pack(i)
    f103allFile.write(epID)
else:
  for i in range(endpointIDStart - endPointIDNum, endpointIDStart):  
    epID = epID_struct.pack(i)
    f103allFile.write(epID)  

#将剩余空间填入0xFF
remain = 1*1024 - 48 - endPointIDNum*2

for i in range(0, remain):
  f103allFile.write(init_remain)


f103allFile.close

print('F103 firmware 生成完毕: OK')

print('生成的文件如下：')
print(f405path)
print(f103path)


