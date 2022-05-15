.globl main

.data
source:
    .word   3
    .word   1
    .word   4
    .word   1
    .word   5
    .word   9
    .word   0
dest:
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0

.text
fun:
    addi t0, a0, 1
    sub t1, x0, a0
    mul a0, t0, t1
    jr ra

main:
    # BEGIN PROLOGUE
    addi sp, sp, -20
    sw s0, 0(sp)
    sw s1, 4(sp)
    sw s2, 8(sp)
    sw s3, 12(sp)
    sw ra, 16(sp)
    # END PROLOGUE
    addi t0, x0, 0 # k
    addi s0, x0, 0 # sum
    la s1, source
    la s2, dest
loop:
    slli s3, t0, 2  # k * 4 in s3
    add t1, s1, s3  # source + k * 4 in t1
    lw t2, 0(t1)    # source[k] in t2
    beq t2, x0, exit # source[k] == 0 => return
    add a0, x0, t2   # source[k] in a0
    addi sp, sp, -8  # add stack size
    sw t0, 0(sp)     # store k
    sw t2, 4(sp)     # store source[k]
    jal fun          # run function
    lw t0, 0(sp)     # restore k in t0
    lw t2, 4(sp)     # restore source[k] in t2
    addi sp, sp, 8   # free stack
    add t2, x0, a0   # fun(source[k]) in t2
    add t3, s2, s3   # dest + k * 4 in t3
    sw t2, 0(t3)     # store fun(source[k]) in dest[k]
    add s0, s0, t2   # sum + fun(source[k]) 
    addi t0, t0, 1   # k + 1 in t0
    jal x0, loop
exit:
    add a0, x0, s0
    # BEGIN EPILOGUE
    lw s0, 0(sp)
    lw s1, 4(sp)
    lw s2, 8(sp)
    lw s3, 12(sp)
    lw ra, 16(sp)
    addi sp, sp, 20
    # END EPILOGUE
    jr ra
