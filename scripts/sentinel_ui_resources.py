"""Explicit resource ownership; never interpret handle closure as mechanical safety."""
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading


class SerialOwner:
    def __init__(self, factory):
        self.factory = factory
        self.handles = []
        self._closing = threading.Event()
        self._lock = threading.RLock()

    def open(self, *args, **kwargs):
        with self._lock:
            if self._closing.is_set():
                raise OSError('页面正在释放资源，不允许重新连接')
            raw = self.factory(*args, **kwargs)
            self.handles.append(raw)
            return SerialHandle(self, raw)

    def request_freeze(self):
        """Block future I/O immediately; do not wait for an in-flight call on the UI thread."""
        self._closing.set()

    def freeze(self):
        self.request_freeze()
        # Wait for an already-started write/open to finish; future writes are blocked.
        with self._lock:
            pass

    def close(self):
        self.freeze()
        errors = []
        with self._lock:
            for raw in self.handles:
                try:
                    if raw.is_open:
                        raw.close()
                except Exception as error:
                    errors.append(str(error))
        if errors:
            raise OSError('; '.join(errors))

    def released(self):
        with self._lock:
            return all(not raw.is_open for raw in self.handles)


class SerialHandle:
    def __init__(self, owner, raw):
        self.owner, self.raw = owner, raw

    def write(self, data):
        with self.owner._lock:
            if self.owner._closing.is_set():
                raise OSError('返回清理中，禁止发送报文')
            return self.raw.write(data)

    def read(self, size=1):
        if self.owner._closing.is_set():
            raise OSError('返回清理中，结束读取')
        return self.raw.read(size)

    def close(self):
        self.raw.close()

    def __getattr__(self, name):
        return getattr(self.raw, name)


class ProcessJob:
    """One headless process group; graceful cancellation, no forced-exit bypass."""
    def __init__(self, command, *, cwd, env=None, hardware=False, input_pipe=False):
        self.needs_confirmation = hardware
        self.structured_output = input_pipe
        self.events = queue.Queue(maxsize=256)
        self.process = subprocess.Popen(command, cwd=cwd, env=env, start_new_session=True,
                                        stdin=subprocess.PIPE if input_pipe else None,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, encoding='utf-8', errors='replace')
        self._reader = threading.Thread(target=self._read, name='sentinel-tool-output', daemon=False)
        self.startup_error = ''
        self._reader_started = False
        try:
            self._reader.start()
            self._reader_started = True
        except RuntimeError as error:
            # Keep ownership of the already-started process, even without a reader.
            self.startup_error = str(error)
            try:
                self.request_stop()
            except OSError as stop_error:
                self.startup_error += '; 取消失败: ' + str(stop_error)
            self.events.put_nowait('后台输出线程启动失败: ' + self.startup_error)

    def _read(self):
        output = self.process.stdout
        assert output is not None
        try:
            for line in output:
                try:
                    text = line.rstrip('\n')
                    self.events.put_nowait(text if self.structured_output else text[:4000])
                except queue.Full:
                    try:
                        self.events.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self.events.put_nowait('[输出过多，已丢弃较早行] ' + line[:1000])
                    except queue.Full:
                        pass
        finally:
            output.close()

    def _group_alive(self):
        try:
            os.killpg(self.process.pid, 0)
        except ProcessLookupError:
            return False
        if self.process.poll() is None:
            return True
        # 已退出的组长不能代表整组退出；僵尸进程则已不持有串口或输出管道。
        try:
            for entry in Path('/proc').iterdir():
                if not entry.name.isdecimal():
                    continue
                try:
                    fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                    if int(fields[2]) == self.process.pid and fields[0] != 'Z':
                        return True
                except (OSError, ValueError, IndexError):
                    continue
            return False
        except OSError:
            return True

    def request_stop(self, kill_after=2.0):
        """先请求协作停止，再在限定时间后强制结束残留进程组."""
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                os.killpg(self.process.pid, signum)
            except ProcessLookupError:
                return
        if kill_after is not None:
            timer = threading.Timer(kill_after, self._kill_if_alive)
            timer.daemon = True
            try:
                timer.start()
            except RuntimeError:
                self._kill_if_alive()

    def _kill_if_alive(self):
        if not self._group_alive():
            return
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def send_line(self, line):
        if self.process.poll() is not None or self.process.stdin is None or self.process.stdin.closed:
            raise OSError('检查连接尚未建立或已经结束')
        self.process.stdin.write(line + '\n')
        self.process.stdin.flush()

    def _close_input(self):
        if self.process.stdin is not None and not self.process.stdin.closed:
            try:
                self.process.stdin.close()
            except BrokenPipeError:
                pass

    def close(self):
        self.request_stop()
        self._close_input()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError('后台进程仍未结束，禁止离开；请等待后重试') from error
        if self._reader_started:
            self._reader.join(timeout=3)
        elif self.process.stdout:
            self.process.stdout.close()
        if not self.released():
            raise RuntimeError('后台进程组或输出线程仍未释放，禁止离开')

    def released(self):
        if self.process.poll() is not None:
            self._close_input()
        output_closed = self.process.stdout is None or self.process.stdout.closed
        return (self.process.poll() is not None and not self._reader.is_alive()
                and output_closed and not self._group_alive())
