#!/usr/bin/env python3
"""Prove the audit fails closed when a tracked MSP setter loses its bracket."""
from pathlib import Path
import shutil, subprocess, tempfile
root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as tmp:
    test = Path(tmp)
    shutil.copytree(root / 'src/main', test / 'src/main')
    p = test / 'src/main/blackbox/blackbox_params.c'
    source = p.read_text()
    for command in ['MSP_SET_SERVO_CONFIG', 'MSP_SET_PID_TUNING', 'MSP_SET_GOVERNOR_CONFIG',
                    'MSP2_ESC_SENSOR_TRIAL', 'MSP2_RX_SERIAL_TRIAL']:
        p.write_text(source.replace('    case ' + command + ':\n', '', 1))
        got = subprocess.run(['python3', str(root / 'src/utils/blackbox_params_audit.py'), str(test / 'src/main')], capture_output=True, text=True)
        assert got.returncode != 0, command + ' was not rejected'
        assert command in got.stdout and 'MISSING BRACKET' in got.stdout, got.stdout
        print(command + ': missing mutation bracket rejected')
    p.write_text(source.replace('    case MSP_SELECT_SETTING:', '    case MSP_BOXIDS:\n    case MSP_SELECT_SETTING:', 1))
    got = subprocess.run(['python3', str(root / 'src/utils/blackbox_params_audit.py'), str(test / 'src/main')], capture_output=True, text=True)
    assert got.returncode != 0 and 'read or passthrough commands instrumented' in got.stdout
    print('MSP_BOXIDS: read request bracket rejected')

    p.write_text(source)
    for name, hook, writer in [
        ('sensors/battery.c', 'blackboxParamsOpBegin(BBP_SRC_PROFILE, 0x200 | pendingBatteryProfile);', 'applyBatteryProfile'),
        ('rx/rx.c', 'blackboxParamsOpBegin(BBP_SRC_TASK, BBP_TASK_RX_SERIAL_TRIAL);', 'rxSerialTrialApplyCombo'),
        ('sensors/esc_sensor.c', 'blackboxParamsOpBegin(BBP_SRC_TASK, BBP_TASK_ESC_SENSOR_TRIAL);', 'escSensorTrialApplyCombo'),
        ('fc/tasks.c', 'blackboxParamsOpBegin(BBP_SRC_TASK, BBP_TASK_SRXL2_ESC);', 'srxl2esc_service'),
    ]:
        writer_file = test / 'src/main' / name
        original = writer_file.read_text()
        assert hook in original, name
        writer_file.write_text(original.replace(hook, '', 1))
        got = subprocess.run(['python3', str(root / 'src/utils/blackbox_params_audit.py'), str(test / 'src/main')],
                             capture_output=True, text=True)
        assert got.returncode != 0, writer + ' was not rejected'
        assert any(line.startswith('GAP') and writer in line for line in got.stdout.splitlines()), got.stdout
        print(writer + ': missing asynchronous writer bracket rejected')
        writer_file.write_text(original)
