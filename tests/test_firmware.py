"""Compile and run the firmware's safety logic on the PC: the limit functions (property test over 2 million inputs
including NaN/inf) and the REAL .ino against Arduino/MobaTools stubs (limits, timeout, malformed lines, tether latch).
Needs a C++ compiler: `pip install ziglang` provides one (`python -m ziglang c++`), or any g++/clang++ on PATH."""
import os
import shutil
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, 'firmware')


def compiler():
    for c in (['g++'], ['clang++'], [sys.executable, '-m', 'ziglang', 'c++']):
        try:
            if c[0] in ('g++', 'clang++'):
                if shutil.which(c[0]):
                    return c
            else:
                subprocess.check_output(c + ['--version'], stderr=subprocess.STDOUT)
                return c
        except Exception:
            continue
    return None


CC = compiler()


def build_and_run(src, out, extra=()):
    exe = os.path.join(FW, out + ('.exe' if os.name == 'nt' else ''))
    for attempt in range(4):             # zig's compiler cache on Windows occasionally fails to open a file: just retry
        r = subprocess.run(CC + ['-w', '-std=c++14', '-O1'] + list(extra) + ['-o', exe, os.path.join(FW, src)], cwd=FW,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if r.returncode == 0:
            break
        time.sleep(1.0)
    else:
        raise RuntimeError('compile failed: ' + r.stdout.decode(errors='replace')[-2000:])
    return subprocess.run([exe], cwd=FW, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)


@pytest.mark.skipif(CC is None, reason='no C++ compiler')
def test_limit_functions_hold_for_any_input():
    r = build_and_run('test_limits.cpp', 'test_limits')
    assert r.returncode == 0, r.stdout.decode()
    assert b'LIMIT TESTS PASSED' in r.stdout


@pytest.mark.skipif(CC is None, reason='no C++ compiler')
def test_real_ino_behaviour_on_host_stubs():
    r = build_and_run('fw_sim.cpp', 'fw_sim', ['-I', os.path.join(FW, 'stubs')])
    assert r.returncode == 0, r.stdout.decode()
    assert b'FIRMWARE TESTS PASSED' in r.stdout
