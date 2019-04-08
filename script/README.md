##endpoint uart test tool

* python版本2.7
* 执行python_serial.py脚本
```
	python python_serial.py /dev/tty.SLAB_USBtoUART 115200 8 'N' 1 13
	参数1:串口名
	参数2:波特率
	参数3:数据位数
	参数4:椒盐味
	参数5:停止位数
	参数6:每一包数据的超时时间
```

* sendcmd.sh
```
	./sendcmd.sh [id] [cmd]
	id:地锁的id，支持十进制和十六进制
	cmd:地锁的控制命令
```

* endpoint_cmd.txt
	```
	测试脚本所需要的中间文件，用来保存控制id和cmd，不能删除
	```

