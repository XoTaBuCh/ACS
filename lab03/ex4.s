.globl iterative
.globl recursive

.data
n: .word 49
m: .word 9

.text
main:
    la t0, n
    lw a0, 0(t0)
    la t0, m
    lw a1, 0(t0)
    jal ra, tester

    addi a1, a0, 0
    addi a0, x0, 1
    ecall # Print Result

    addi a1, x0, '\n'
    addi a0, x0, 11
    ecall # Print newline

    addi a0, x0, 10
    ecall # Exit

tester:
    addi sp, sp, -4
    sw ra, 0(sp) 
    add s3, zero, a0 # save n
    add s4, zero, a1 # save m
    jal ra, iterative
    add s0, zero, a0
    add a0, zero, s3 #restore n
    add a1, zero, s4 #restore m
    jal ra, recursive
    beq a0, s0, exit
    addi a0, zero, -1
    j exit
    exit:
    lw ra, 0(sp) 
    addi sp, sp, -4
    jr ra

iterative:
    mod_it:
        
    loop:
        blt a0, a1, mod_exit
        sub a0, a0, a1
        j loop

    mod_exit:
        jr ra
    
recursive:
    addi sp, sp, -4 # more stack space for return address
    sw ra, 0(sp) # store return address
    blt a0, a1, return # if n <= m return 
    sub a0, a0, a1 # n = n - m
    jal recursive # call recursive and go back (r0 contains n % m)
    
    return:
    lw ra, 0(sp) # restore return address
    addi sp, sp, 4 # restore stack space
    jr ra
