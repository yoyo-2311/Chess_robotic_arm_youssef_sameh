import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from dual_scara_desktop.supervisor import ProcessSupervisor


def wait(predicate,timeout=8):
    until=time.monotonic()+timeout
    while not predicate() and time.monotonic()<until:time.sleep(.02)
    assert predicate()


def test_switch_stop_and_close_only_owned_groups(tmp_path):
    unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True)
    s=ProcessSupervisor(tmp_path,10000+os.getpid()*2,grace=.2)
    try:
        # Both launch leader and its child ignore SIGINT; supervisor must escalate and reap them.
        script='import signal,subprocess,sys,time; signal.signal(signal.SIGINT,signal.SIG_IGN); subprocess.Popen([sys.executable,"-c","import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);time.sleep(60)"],start_new_session=True); print("ready",flush=True); time.sleep(60)'
        s.start([sys.executable,'-u','-c',script],1);wait(lambda:s.running)
        time.sleep(.2);s.stop();wait(lambda:not s.running)
        assert unrelated.poll() is None
        s.start([sys.executable,'-c','import time; time.sleep(60)'],2);wait(lambda:s.running)
        s.shutdown();wait(lambda:not s.alive())
        assert unrelated.poll() is None
    finally:
        s.shutdown();unrelated.terminate();unrelated.wait(timeout=5)


def test_second_app_cannot_start_mode_on_owned_domain(tmp_path):
    first=ProcessSupervisor(tmp_path/'a',10001+os.getpid()*2,grace=.2);second=ProcessSupervisor(tmp_path/'b',10001+os.getpid()*2,grace=.2)
    try:
        first.start([sys.executable,'-c','import time;time.sleep(60)'],1);wait(lambda:first.running)
        second.start([sys.executable,'-c','import time;time.sleep(60)'],1)
        errors=[]
        def error_received():
            while not second.events.empty():
                kind,data=second.events.get()
                if kind=='error':errors.append(data)
            return bool(errors)
        wait(error_received);assert not second.running and first.running
    finally:
        first.shutdown();second.shutdown();wait(lambda:not first.alive() and not second.alive())
