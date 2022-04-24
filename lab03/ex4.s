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
	mod_rec:
		addi    sp, sp, -16
        sw      ra, 12(sp)                      
        sw      s0, 8(sp)                      
        addi    s0, sp, 16
        sw      a0, -12(s0)
        sw      a1, -16(s0)
        lw      a1, -12(s0)
        lw      a0, -16(s0)
        bge     a0, a1, flag2
        j       flag1
	flag1:
        lw      a0, -12(s0)
        lw      a1, -16(s0)
        sub     a0, a0, a1
        call    mod_rec
        sw      a0, -12(s0)
        j       flag2
	flag2:
        lw      a0, -12(s0)
        lw      ra, 12(sp)                     
        lw      s0, 8(sp)                       
        addi    sp, sp, 16
        ret
