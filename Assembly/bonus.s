.data
    n: .word 11
.text
.globl __start


jal x0, __start
#----------------------------------------------------Do not modify above text----------------------------------------------------
FUNCTION:
    # Initialize
    addi t0, a0, 0 		# set t0 = n
    addi t1, x0, 0 		# set t1 = 0
    addi sp, sp, -8
    sw   ra, 0(sp)
    
    # Calculate Result
    jal  x1, SUBFUNC
    add  a1, x0, t1
    
    # Retrieve & return
    lw ra, 0(sp)
    addi sp, sp , 8
    jalr x0, 0(ra)
    
SUBFUNC:
    addi sp, sp, -8
    sw ra, 0(sp)
    beq t0, x0, CASE_1
    addi t2, x0, 10
    bge  t0, t2, CASE_3
    jal  x0, CASE_2
RETURN:
    lw ra, 0(sp)
    addi sp, sp, 8
    jalr x0, 0(ra)
        
# Different Cases
# n = 0
CASE_1:
    addi t1, x0, 7
    jal x0, RETURN
    
# 1 <= n < 10
CASE_2:
    addi t0, t0, -1
    jal  x1, SUBFUNC
    slli t1, t1, 1
    jal  x0, RETURN
    
# n >= 10
CASE_3:
    slli t2, t0, 2	# t2 = 4n
    slli t3, t0, 1	# t3 = 2n
    add  t4, t2, t3	# t4 = 6n
    add  t5, t0, t4	# t5 = 7n
    srli a2, t5, 3	# a2 = 7/8n
    add  t5, t0, t3	# t5 = 3n
    srli t0, t5, 2	# t0 = 3n/4
    addi sp, sp, -8
    sw   a2, 0(sp)
    jal  x1, SUBFUNC
    lw   a2, 0(sp)
    addi sp, sp , 8
    slli t1, t1, 1
    add  t1, a2, t1
    addi t1, t1, -137
    jal  x0, RETURN
    
# Todo: Define your own function
# We store the input n in register a0, and you should store your result in register a1

#----------------------------------------------------Do not modify below text----------------------------------------------------
__start:
    la   t0, n
    lw   a0, 0(t0)
    jal  x1, FUNCTION
    la   t0, n
    sw   a1, 4(t0)
    li 	 a7, 10
    ecall
