#!/usr/bin/env python
#-*- coding:utf:8 -*-

import serial
import threading
import time
import select
import sys
import Queue
import time

serial_name = ''
serial_baud = 0
serial_databits = 0
serial_parity = ''
serial_stopbits = 0
serial_recv_timeout = 0

def InverUint8(dat):
    temp = 0
    for i in range(8):
        if dat & (1 << i) > 0:
            shift_bit = 7 - i
            temp |= 1 << shift_bit
    return temp

def InverUint16(dat):
    temp = 0
    for i in range(16):
        if dat & (1 << i) > 0:
            shift_bit = 15 - i
            temp |= 1 << shift_bit
    return temp

def calc_senddata(id, cmd):
    ret_list = []
    ret_list.append(id / 0x100)
    ret_list.append(id % 0x100)
    ret_list.append(id % 30)
    ret_list.append(0xa5)
    ret_list.append(0x0c)
    ret_list.append(id % 0x100)
    ret_list.append(id / 0x100)
    ret_list.append(cmd)
    ret_list.append(0x01)
    ret_list.append(0x02)
    ret_list.append(0x03)
    ret_list.append(0x04)
    wCRCin = 0x0000
    wCPoly = 0x8005
    wChar = 0
    pos = 3
    while True:
        wChar = ret_list[pos]
        #print 'wChar1:%s'%hex(wChar)
        wChar = InverUint8(wChar)
        wChar = wChar << 8
        #print 'wChar2:%s'%hex(wChar)
        wCRCin = wCRCin^wChar
        #print 'wCRCin1:%s'%hex(wCRCin)
        for i in range(8):
            if (wCRCin & 0x8000) > 0:
                wCRCin = wCRCin << 1
                wCRCin = wCRCin ^ wCPoly
            else:
                wCRCin = wCRCin << 1
        #print 'wCRCin2:%s'%hex(wCRCin)
        pos = pos + 1
        if pos >= 12:
            break
    wCRCin = InverUint16(wCRCin)
#print hex(wCRCin)
    ret_list.append(wCRCin % 0x100)
    ret_list.append(wCRCin / 0x100)
    ret_list.append(0x5a)
    return ret_list
    

def read_cmd_from_file():
    id = 0
    cmd = 0
    ret = []
    try:
        with open('endpoint_cmd.txt', 'rb+') as file_obj:
            for line in file_obj:
                if line.startswith('id'):
                    msg = line.split('=')
#print msg[1]
                    if msg[1].startswith('0x'):
                        id = int(msg[1], 16)
                    else:
                        id = int(msg[1], 10)
                    ret.append(id)
                if line.startswith('cmd'):
                    msg = line.split('=')
                    cmd = int(msg[1], 10)
                    ret.append(cmd)
                    file_obj.seek(0)
                    file_obj.truncate()
                    #return ret    
#           contents = file_obj.read()
#           print contents
    except Exception as e:
        print e.args
    finally:
        #print 'read_cmd_from_file end'
        return ret

def serial_data_process(name, baud, databits, parity, stopbits, timeout):
    ser = serial.Serial(name, baudrate=baud, bytesize=databits, parity=parity, stopbits=stopbits, timeout=1)
    print(ser)
    inputs = [ser,]
    outputs = []
    msglen = 0
    ch_list = []
    timeout_count = 0
    message_queue = {}
    message_queue[ser] = Queue.Queue()
    recv_count = 0
    all_count = 0

    while True:
        readable,writeable,exceptional = select.select(inputs, outputs, inputs, 1)
        if not (readable or writeable or exceptional):
            timeout_count = timeout_count + 1
            msg_list = read_cmd_from_file()
            if msg_list:
                num = len(msg_list)
#print num
                if num == 2:
                    #print 'id=%d,cmd=%d' % (msg_list[0], msg_list[1])
                    message_queue[ser].put(msg_list)
                    outputs.append(ser)

            if timeout_count > timeout:
#print('timeout:%d'%timeout_count)
                timeout_count = 0
                all_count = all_count + 1
                print 'lost %d message' % (all_count - recv_count)
                
            continue
        for r in readable:
            ch = r.read(1).encode('hex')
#           print 'recv:%s'%ch
            if ch == 'a5':
#                print 'find msg head'
                msglen = 1 
            if msglen > 0:
                ch_list.append(ch)
                msglen = msglen + 1 
            if msglen == 14: 
                msglen = 0
                timeout_count = 0
                print time.strftime('%H:%M:%S',time.localtime(time.time())),
                for val in ch_list:
                    print(val),
                print('lost:'),
                print all_count - recv_count
#                print(" ")
                recv_count = recv_count + 1
                all_count = all_count + 1
                ch_list = []
        for w in writeable:
#print w
            try:
                msg = message_queue[w].get_nowait()
            except Queue.Empty:
                outputs.remove(w)
            except Exception as e:
                print e.args
                if w in outputs:
                    outputs.remove(w)
            else:
                try:
                    send_id = msg[0]
                    send_cmd = msg[1]
                    send_buff = calc_senddata(send_id, send_cmd)
                    print 'serial send:',
                    for val in send_buff:
                        print hex(val),
                        w.write(chr(val))
                    print(" ")
                    outputs.remove(w)
                except Exception as e:
                    print e.args


def helpmsg():
    print('help message:')
    print('paramter must be 7')
    print('python xxx.py serial_name baud databits parity stopbits recv_timeout')

if __name__ == '__main__':
#    recv_thread = threading.Thread(target=serial_read_poll, args=(ser,))
#    recv_thread.setDaemon(True)
#    recv_thread.start()
#   0是文件名
    num = len(sys.argv)
    print(num)
    if num < 7:
        helpmsg()
        exit()

    serial_name = sys.argv[1]
    serial_baud = int(sys.argv[2], 10)
    serial_databits = int(sys.argv[3], 10)
    serial_parity = sys.argv[4]
    serial_stopbits = int(sys.argv[5], 10)
    serial_recv_timeout = int(sys.argv[6], 10)
    serial_data_process(serial_name, serial_baud, serial_databits, serial_parity, serial_stopbits, serial_recv_timeout)
#ret = calc_senddata(0x90, 1)
#   print ret

