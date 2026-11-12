# Only read live CAN status registers. No halt, reset, resume, register write or motor commands.
init
set before [stm32f4x.cpu curstate]
echo "CPU_STATE_BEFORE $before"
if {$before ne "running"} {
    echo "ABORT_NOT_RUNNING"
    shutdown error
}
set mcr1 [lindex [stm32f4x.cpu read_memory 0x40006400 32 1] 0]
set mcr2 [lindex [stm32f4x.cpu read_memory 0x40006800 32 1] 0]
echo "CAN1_MCR $mcr1"
echo "CAN2_MCR $mcr2"
for {set i 0} {$i < 32} {incr i} {
    set tsr1 [lindex [stm32f4x.cpu read_memory 0x40006408 32 1] 0]
    set tsr2 [lindex [stm32f4x.cpu read_memory 0x40006808 32 1] 0]
    set esr2 [lindex [stm32f4x.cpu read_memory 0x40006818 32 1] 0]
    echo "CAN_SAMPLE $i $tsr1 $tsr2 $esr2"
    after 5
}
echo "CPU_STATE_AFTER [stm32f4x.cpu curstate]"
shutdown
