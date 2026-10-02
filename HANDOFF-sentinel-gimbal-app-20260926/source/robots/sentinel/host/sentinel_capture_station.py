"""实车只读采集入口. 检查接线与连续记录互斥使用同一个串口, 不控制电机."""
import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk,messagebox
from serial.tools import list_ports
import sentinel_stream as stream
import sentinel_capture

AXES={'静止 / 温漂':'stationary','主yaw单轴运动':'main','发射yaw单轴运动':'fire','pitch单轴运动':'pitch'}


class Station:
    def __init__(self,root,directory):
        self.root=root;self.directory=Path(directory);self.events=queue.Queue()
        self.worker=None;self.inspector=None;self.stop_event=threading.Event();self.closing=False
        root.title('Sentinel · 实车只读连续采集');root.geometry('780x510')
        frame=ttk.Frame(root,padding=18);frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='只采集, 不驱动、不使能、不置零电机',font=('Microsoft YaHei UI',17,'bold')).pack(anchor='w')
        ttk.Label(frame,text='先刷本包 capture 固件. 机构无驱动输出, pitch需支撑. 不触碰机械限位/拉紧线束.').pack(anchor='w',pady=8)
        row=ttk.Frame(frame);row.pack(fill='x',pady=6)
        self.port=tk.StringVar(value='COM3')
        ttk.Label(row,text='串口').pack(side='left')
        self.ports=ttk.Combobox(row,textvariable=self.port,width=12);self.ports.pack(side='left',padx=6)
        ttk.Button(row,text='刷新',command=self.scan).pack(side='left')
        self.check_button=ttk.Button(row,text='1. 检查电机 / IMU',command=self.inspect);self.check_button.pack(side='left',padx=12)
        ttk.Label(row,text='固定1,000,000 baud').pack(side='left')
        row=ttk.Frame(frame);row.pack(fill='x',pady=6)
        self.axis=tk.StringVar(value='静止 / 温漂');self.seconds=tk.StringVar(value='30')
        ttk.Label(row,text='采集内容').pack(side='left')
        ttk.Combobox(row,textvariable=self.axis,values=list(AXES),state='readonly',width=21).pack(side='left',padx=6)
        ttk.Label(row,text='时长(s)').pack(side='left');ttk.Entry(row,textvariable=self.seconds,width=7).pack(side='left',padx=6)
        ttk.Label(frame,text='备注: 其它轴保持的姿态、正反方向、供电/安装变化等').pack(anchor='w',pady=(8,2))
        self.note=tk.StringVar();ttk.Entry(frame,textvariable=self.note).pack(fill='x')
        row=ttk.Frame(frame);row.pack(fill='x',pady=14)
        self.start_button=ttk.Button(row,text='2. 开始记录',command=self.start);self.start_button.pack(side='left')
        ttk.Button(row,text='停止记录',command=self.stop_event.set).pack(side='left',padx=10)
        ttk.Button(row,text='打开记录目录',command=self.open_directory).pack(side='left')
        self.status=tk.StringVar(value='未记录. 检查窗口关闭后再开始; 看到“正在记录”才手动运动.')
        ttk.Label(frame,textvariable=self.status,font=('Microsoft YaHei UI',12),wraplength=725).pack(anchor='w',pady=8)
        self.detail=tk.StringVar(value='运动时尽量只转所选关节, 其它关节保持不动; 电机与IMU同时记录.')
        ttk.Label(frame,textvariable=self.detail,wraplength=725).pack(anchor='w',pady=8)
        ttk.Label(frame,text='记录器停止不等于电机停车. 没有一圈要求, 只在已确认机械行程内缓慢往返.',foreground='#8a4a00').pack(anchor='w',pady=8)
        root.protocol('WM_DELETE_WINDOW',self.close);root.after(100,self.update)
        self.scan()

    def scan(self):
        self.ports['values']=[p.device for p in list_ports.comports()]

    def inspect(self):
        if self.worker and self.worker.is_alive():return
        if self.inspector and self.inspector.poll() is None:return
        self.directory.mkdir(parents=True,exist_ok=True)
        args=['--inspect','--port',self.port.get(),'--output',str(self.directory/f'mapping-{time.time_ns()}.jsonl')]
        command=[sys.executable] if getattr(sys,'frozen',False) else [sys.executable,str(Path(__file__).resolve())]
        self.inspector=subprocess.Popen(command+args)
        self.status.set('检查各电机编码器与IMU; 检查窗口关闭后再开始连续记录.')

    def start(self):
        if self.worker and self.worker.is_alive():return
        if self.inspector and self.inspector.poll() is None:
            self.status.set('请先关闭电机/IMU检查窗口, 释放同一个串口.');return
        try:
            seconds=float(self.seconds.get())
            if not 1<=seconds<=1800:raise ValueError('时长范围1..1800秒')
            axis=AXES[self.axis.get()]
            profile='stationary' if axis=='stationary' else 'rotation'
            self.directory.mkdir(parents=True,exist_ok=True)
            path=self.directory/f'{axis}-{time.time_ns()}.sraw'
        except (ValueError,OSError,KeyError) as exc:
            messagebox.showerror('未开始',str(exc));return
        self.stop_event.clear();port=self.port.get();note=self.note.get()
        self.status.set('正在连接与核对固件, 暂未开始记录…')
        def work():
            try:stream.collect(port,path,seconds,profile,note,axis,self.stop_event,self.events.put)
            except Exception as exc:self.events.put(dict(event='error',message=str(exc)))
        self.worker=threading.Thread(target=work,name='capture-serial',daemon=False);self.worker.start()

    def open_directory(self):
        self.directory.mkdir(parents=True,exist_ok=True);os.startfile(self.directory)

    def update(self):
        while not self.events.empty():
            event=self.events.get_nowait();kind=event['event']
            if kind=='recording':
                self.status.set('正在记录 — 现在开始缓慢手动运动; 静止项目请保持不动.')
                self.detail.set(f'数据文件: {event["path"]}')
                self.root.bell()
            elif kind=='progress':
                self.status.set(f'正在记录: {event["records"]}条 | 板端丢弃{event["dropped"]} | 记录缺口{event["gaps"]} | CRC错误{event["crc_errors"]}')
            elif kind=='finished':
                result=event['summary'];counts=result['received']
                self.status.set('记录结束 · 传输完整' if result['complete'] else '记录结束 · 有缺口/异常, 请保留文件并反馈')
                if result.get('source_issues') or result.get('diagnostics_error'):
                    self.status.set('记录结束 · 板端诊断有异常或无法读取, 请保留文件并反馈')
                self.detail.set(f'{event["path"]}\n陀螺仪 {counts.get("gyro:0",0)} / 加速度 {counts.get("accel:0",0)} / '
                                f'主yaw {counts.get("can:4",0)} / 发射yaw {counts.get("can:5",0)} / pitch {counts.get("can:9",0)}\n'
                                + ('静止项目只记录IMU, 不记录电机. 传输完整不代表标定通过.' if result.get('profile')=='stationary' else
                                   '传输完整不代表外设齐全或标定通过. 为0的轴需要检查接线/ID.'))
            elif kind=='error':self.status.set('采集未完成: '+event['message'])
        active=bool(self.worker and self.worker.is_alive())
        self.start_button.configure(state='disabled' if active else 'normal')
        self.check_button.configure(state='disabled' if active else 'normal')
        if self.closing and not active:self.root.destroy();return
        self.root.after(100,self.update)

    def close(self):
        self.stop_event.set();self.closing=True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='COM3');parser.add_argument('--inspect',action='store_true')
    parser.add_argument('--output',type=Path);parser.add_argument('--self-test',type=Path)
    parser.add_argument('--capture',action='store_true');parser.add_argument('--seconds',type=float,default=30)
    args=parser.parse_args()
    if args.self_test:
        root=tk.Tk();root.withdraw();root.update();root.destroy()
        args.self_test.write_text(json.dumps({'baud':stream.BAUD,'build':stream.BUILD,'tk':'ok','serial':'imported'}),encoding='utf-8');return
    if args.inspect:
        sys.argv=[sys.argv[0],'--port',args.port,'--baud',str(stream.BAUD),'--log',str(args.output)]
        sentinel_capture.main();return
    if args.capture:
        stream.collect(args.port,args.output,args.seconds);return
    directory=(Path(sys.executable).resolve().parent.parent if getattr(sys,'frozen',False) else Path.cwd())/'records'
    root=tk.Tk();Station(root,directory);root.mainloop()


if __name__=='__main__':main()
