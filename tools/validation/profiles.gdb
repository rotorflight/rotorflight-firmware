# Run with the profiling ELF after attaching your usual SWD/OpenOCD server.
# Samples include interrupt latency. Indices: HeaderTick, WriteHeader, Running, AfterFrame, OpBegin, OpEnd, OperationSpan.
# OperationSpan includes the writer and loaders; use separate scenarios for EEPROM saves and ordinary changes.
# Set the CPU clock from the actual board before converting cycles to time.
set pagination off
p SystemCoreClock
p bbpProfileCalls
p bbpProfileMaxCycles
p bbpFlashfsDroppedBytes
p usedStackSize
p/x &_estack
p/d (unsigned long)&_Min_Stack_Size
# Reset between OFF, CHANGES and FULL measurements, while the bench setup is disarmed.
# set var bbpProfileCalls = {0,0,0,0,0,0,0}
# set var bbpProfileMaxCycles = {0,0,0,0,0,0,0}
# set var bbpFlashfsDroppedBytes = 0
