#!/usr/bin/env python
#encoding:utf-8


# ----------------------------------------
# 语言：Python2.7
# 功能：socket客户端
# 日期：2020-07-23
# ----------------------------------------

import sys
defaultencoding = 'utf-8'
if sys.getdefaultencoding() != defaultencoding:
    reload(sys)
    sys.setdefaultencoding(defaultencoding)

from socket import *

HOST = 'juyingtech.tpddns.cn'#'129.211.44.129'#'192.168.2.227'#juyingtech.tpddns.cn'  # 连接地址
PORT = 21567        # 端口
BUFSIZ =1024        # 接收数据大小
ADDR = (HOST,PORT)

tcpCliSock = socket(AF_INET,SOCK_STREAM)
tcpCliSock.connect(ADDR)

def socketSend(data):
    tcpCliSock.send(data)
# tcpCliSock.close()
