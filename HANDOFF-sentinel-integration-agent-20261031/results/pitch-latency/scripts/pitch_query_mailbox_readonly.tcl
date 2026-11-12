# Read-only CAN2 mailbox attribution. No reset/halt/resume/register writes.
init
set before [stm32f4x.cpu curstate]
echo "CPU_STATE_BEFORE $before"
if {$before ne "running"} { shutdown error }
set mcr [lindex [stm32f4x.cpu read_memory 0x40006800 32 1] 0]
echo "CAN2_MCR $mcr"
for {set i 0} {$i < 24} {incr i} {
    set a [lindex [stm32f4x.cpu read_memory 0x40006808 32 1] 0]
    set tir [lindex [stm32f4x.cpu read_memory 0x400069a0 32 1] 0]
    set tdtr [lindex [stm32f4x.cpu read_memory 0x400069a4 32 1] 0]
    set low [lindex [stm32f4x.cpu read_memory 0x400069a8 32 1] 0]
    set b [lindex [stm32f4x.cpu read_memory 0x40006808 32 1] 0]
    echo "MAILBOX2_SAMPLE $i $a $tir $tdtr $low $b"
    after 5
}
echo "CPU_STATE_AFTER [stm32f4x.cpu curstate]"
shutdown
