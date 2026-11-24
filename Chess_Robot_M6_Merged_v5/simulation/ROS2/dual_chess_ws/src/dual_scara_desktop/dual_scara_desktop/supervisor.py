"""Own process groups only; nonblocking start/stop and bounded log buffering."""
import collections
import ctypes
import errno
import fcntl
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid
from .environment import local_environment


class ProcessSupervisor:
    def __init__(self, log_dir, domain, grace=8.):
        self.log_dir=Path(log_dir)
        self.log_dir.mkdir(parents=True,exist_ok=True)
        self.domain=domain
        self.grace=grace
        self.events=queue.SimpleQueue()
        self.logs=collections.deque(maxlen=2000)
        self.requests=queue.Queue()
        self.running=False
        self.current=None
        self._thread=threading.Thread(target=self._loop,daemon=True)
        # Reap orphaned descendants of our own launch groups, never unrelated children.
        ctypes.CDLL(None,use_errno=True).prctl(36,1,0,0,0)  # Linux PR_SET_CHILD_SUBREAPER
        self._thread.start()

    def start(self, argv, generation):self.requests.put(('start',argv,generation))
    def stop(self):self.requests.put(('stop',))
    def shutdown(self):self.requests.put(('quit',))
    def alive(self):return self._thread.is_alive()

    @staticmethod
    def _group_exists(pgid):
        try:os.killpg(pgid,0);return True
        except ProcessLookupError:return False

    @staticmethod
    def _processes():
        result={}
        for path in Path('/proc').iterdir():
            if not path.name.isdigit():continue
            try:
                text=(path/'stat').read_text();tail=text[text.rfind(')')+2:].split()
                result[int(path.name)]=(int(tail[1]),tail[19])  # parent PID and start time (reuse guard)
            except (OSError,ValueError,IndexError):pass
        return result

    def _loop(self):
        proc=None;pgid=None;lock=None;stopping=False;phase=0;deadline=0;quit_requested=False;log_file=None;owned={}
        def signal_group(sig):
            if pgid:
                try:os.killpg(pgid,sig)
                except ProcessLookupError:pass
            snapshot=self._processes()
            for pid,start in list(owned.items()):
                if pid in snapshot and snapshot[pid][1]==start:
                    try:os.kill(pid,sig)
                    except ProcessLookupError:pass
        while True:
            try:
                cmd=self.requests.get(timeout=.04)
            except queue.Empty:cmd=None
            if cmd and cmd[0]=='start':
                if proc is not None:
                    self.events.put(('error','Stop the current app-managed mode first.'))
                    continue
                try:
                    flags=os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW
                    lock=os.fdopen(os.open(f'/tmp/dual-scara-desktop-{os.getuid()}-{self.domain}.lock',flags,0o600),'w')
                    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    path=self.log_dir/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]+'.log')
                    log_file=open(path,'w',buffering=1)
                    env=local_environment(self.domain)
                    env['ROS_LOG_DIR']=str(self.log_dir/'ros')
                    for key in ['QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH']:env.pop(key,None)
                    proc=subprocess.Popen(cmd[1],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
                                          errors='replace',bufsize=1,start_new_session=True,env=env)
                    pgid=proc.pid;owned={};self.running=True;self.current=cmd[2]
                    stopping=False;phase=0
                    self.events.put(('started',{'generation':cmd[2],'pid':pgid,'log':str(path)}))
                    def read_output(stream, output, generation=cmd[2]):
                        for line in stream:
                            output.write(line);self.logs.append((generation,line.rstrip()))
                        stream.close();output.close()
                    threading.Thread(target=read_output,args=(proc.stdout,log_file),daemon=True).start()
                except Exception as exc:
                    if lock:lock.close();lock=None
                    if log_file and not proc:log_file.close()
                    self.events.put(('error',f'Cannot start mode: {exc}'))
            elif cmd and cmd[0] in ['stop','quit']:
                quit_requested |= cmd[0]=='quit'
                if proc is not None and not stopping:
                    stopping=True;phase=0;deadline=time.monotonic()+self.grace
                    signal_group(signal.SIGINT);self.events.put(('stopping','Stopping owned ROS launch group…'))
                elif proc is None:self.events.put(('stopped',{'code':None}))
            if proc:
                snapshot=self._processes()
                parents={proc.pid,*owned}
                changed=True
                while changed:
                    changed=False
                    for pid,(parent,start) in snapshot.items():
                        if parent in parents and pid not in parents:
                            parents.add(pid);owned[pid]=start;changed=True
                for pid,start in list(owned.items()):
                    if pid not in snapshot or snapshot[pid][1]!=start:owned.pop(pid,None)
                    else:
                        try:
                            reaped,_=os.waitpid(pid,os.WNOHANG)
                            if reaped:owned.pop(pid,None)
                        except ChildProcessError:pass
                code=proc.poll()
                if code is not None:
                    # Popen has reaped the launch leader; reap any adopted children in this group.
                    try:
                        while os.waitpid(-pgid,os.WNOHANG)[0]:pass
                    except ChildProcessError:pass
                    if not stopping and (self._group_exists(pgid) or owned):
                        stopping=True;phase=0;deadline=time.monotonic()+self.grace;signal_group(signal.SIGINT)
                    if not self._group_exists(pgid) and not owned:
                        self.events.put(('stopped',{'code':code,'unexpected':not stopping and code!=0}))
                        proc=None;pgid=None;self.running=False;self.current=None
                        if lock:lock.close();lock=None
                if proc and stopping and time.monotonic()>deadline:
                    phase+=1;signal_group(signal.SIGTERM if phase==1 else signal.SIGKILL)
                    deadline=time.monotonic()+2.
                    self.events.put(('stopping','Escalating shutdown of owned processes only.'))
            if quit_requested and proc is None:break
        self.running=False
