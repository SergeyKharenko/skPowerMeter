import struct
from socket import *
import csv
import threading

PMETER_L2_SIZE=8*1024*1024
DATA_FILENAME="data.csv"

program_running=True
cache_l1=bytearray(2048)
cache_l2=bytearray(0)
cache_l2_s=bytearray(0)
cache_l2_len=0

event=threading.Event()

def net_task(name):
    sock=socket(AF_INET, SOCK_DGRAM)
    bind_addr=("",5590)
    sock.bind(bind_addr)
    sock.settimeout(10)

    data_len=0
    total_len=0

    global cache_l2_len
    global cache_l2_s
    global program_running

    while program_running:
        try:
            data_len=sock.recv_into(cache_l1)
        except timeout:
            continue

        if data_len%4!=0:
            continue
        cache_l2.extend(cache_l1[:data_len])
        total_len+=data_len

        if total_len>=PMETER_L2_SIZE:
            cache_l2_len=total_len
            cache_l2_s=cache_l2
            total_len=0
            event.set()

    cache_l2_len = total_len
    cache_l2_s = cache_l2
    event.set()
    sock.close()

def file_task(name):
    fil=open(DATA_FILENAME,"a",encoding="utf-8")
    csv_w=csv.writer(fil)

    global program_running
    while program_running:
        event.wait()
        if cache_l2_len==0:
            continue

        data_arr=struct.unpack('f',cache_l2_s)
        for i in range(cache_l2_len):
            csv_w.writerow([data_arr[i]])

    fil.close()

def main():
    net_thread=threading.Thread(target=net_task, args=('network',))
    file_thread=threading.Thread(target=file_task, args=('file',))

    print("Press key to Start:")
    input()
    print("Task Started")

    net_thread.start()
    file_thread.start()

    print("Press key to Stop:")
    input()

    global program_running
    program_running=False
    net_thread.join()
    file_thread.join()


if __name__ == '__main__':
    main()
