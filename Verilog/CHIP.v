// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I
    );
    //==== I/O Declaration ========================
    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;

    //==== Reg/Wire Declaration ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    reg  [31:0] reg_rd_data;
    reg  [31:0] reg_sw;
    reg  [31:0] reg_lw;
    wire [31:0] pc;
    wire [6:0]  function7;
    wire [2:0]  function3;
    wire [6:0]  control;
    wire [3:0]  ALU_function;
    wire [3:0]  ALU_control;
    wire [1:0]  mode;
    reg         RegWrite;
    reg         MemRead;
    reg         MemWrite;
    reg  [1:0]  Mode;
    reg         mulDiv_Mode;
    reg  [1:0]  ALUOp;
    reg  [31:0] Immediate;
    reg  [31:0] ALU_result;
    reg  [31:0] Mem_D;
    reg         Valid;
    wire [31:0] alu_result;
    wire [31:0] PC_immediate;
    wire        valid;
    wire        ready;
    wire [31:0] mulDiv_out;
    wire        mulDiv_mode;

    parameter S_format = 7'b0100011;
    parameter I_format = 7'b0010011;
    parameter R_format = 7'b0110011;
    parameter U_format = 7'b0010111;
    parameter SB_format = 7'b1100011;
    parameter JAL = 7'b1101111;
    parameter JALR = 7'b1100111;
    parameter lw = 7'b0000011;
    parameter sw = 7'b0100011;
    parameter add = 4'b0010;
    parameter subtract = 4'b0110;
    parameter AND = 4'b0000;
    parameter OR = 4'b0001;

    assign mem_wen_D = MemWrite;
    assign mem_addr_D = (control == sw)? reg_sw : reg_lw;
    assign mem_wdata_D = rs2_data;
    assign mem_addr_I = PC;

    assign control = mem_rdata_I[6:0];   
    assign function7 = mem_rdata_I[31:25];
    assign function3 = mem_rdata_I[14:12];
    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign rd  = mem_rdata_I[11:7];
    assign mode = Mode;
    assign PC_immediate = Immediate;
    assign alu_result = ALU_result;
    assign pc = PC;
    assign regWrite = RegWrite;
    assign rd_data = (control == R_format && function7 == 7'b0000001)? mulDiv_out : reg_rd_data;
    assign valid = Valid;
    assign mulDiv_mode = mulDiv_Mode;

    
    //==== Submodule Connection ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Todo: other submodules
    PC_control my_PC( //PC,PC_nxt,immediate,alu_result,mode,halt
        .PC(pc),
        .PC_nxt(PC_nxt),
        .immediate(PC_immediate),
        .alu_result(alu_result),
        .mode(mode),
        .halt(valid));

    mulDiv my_mul_Div(//clk, rst_n, valid, ready, mode, in_A, in_B, out
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .ready(ready),
        .mode(mulDiv_mode),
        .in_A(rs1_data),
        .in_B(rs2_data),
        .out(mulDiv_out));
    //==== Combinational Part =====================

    // Todo: any combinational/sequential circuit
    always @(*) begin
        case (control)
            R_format: begin
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                ALUOp = 2'b10;
                Mode = 3;
                ALU_result = 0;
                Immediate = 0;
                reg_sw = 0;
                reg_lw = 0;
                case (function7)   
                    7'b0000000: begin
                        Valid = 0;
                        mulDiv_Mode = 0;
                        case (function3)
                            3'b000: begin //add
                                reg_rd_data = rs1_data + rs2_data;
                            end
                            3'b100: begin //xor
                                reg_rd_data = rs1_data ^ rs2_data;
                            end
                            default: begin
                                reg_rd_data = 0;
                            end
                        endcase
                    end

                    7'b0100000: begin
                        Valid = 0;
                        mulDiv_Mode = 0;
                        case (function3)
                            3'b000: begin //sub
                                reg_rd_data = rs1_data - rs2_data;
                            end
                            default: begin
                                reg_rd_data = 0;
                            end
                        endcase
                    end

                    7'b0000001: begin
                        reg_rd_data = 0;
                        if (ready == 0) begin
                            Valid = 1;
                        end
                        else begin
                            Valid = 0;
                        end
                        case (function3)
                            3'b000: begin
                                mulDiv_Mode = 0;
                            end
                            3'b100: begin
                                mulDiv_Mode = 1;
                            end
                            default: begin
                                mulDiv_Mode = 0;
                            end
                        endcase   
                    end
                    default: begin
                        Valid = 0;
                        mulDiv_Mode = 0;
                        reg_rd_data = 0;
                    end
                endcase
            end

            I_format: begin
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                ALUOp = 2'b00;
                Mode = 3;
                ALU_result = 0;
                reg_sw = 0;
                reg_lw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                Immediate = mem_rdata_I[31:20];
                case (function3)
                    3'b001: begin//slli
                        reg_rd_data = rs1_data << Immediate;
                    end
                    3'b101: begin//srli
                        reg_rd_data = rs1_data >> Immediate;
                    end
                    3'b010: begin //slti //need to be revised
                        if (mem_rdata_I[31] == 0 && rs1_data[31] == 1) begin
                            reg_rd_data = 1;
                        end
                        else if (mem_rdata_I[31] == 1 && rs1_data[31] == 0) begin
                            reg_rd_data = 0;
                        end
                        else if (mem_rdata_I[31] == 0) begin
                            reg_rd_data = (Immediate > rs1_data)? 1:0;
                        end
                        else begin
                            reg_rd_data = ({20'b11111111111111111111, Immediate} > rs1_data)? 1:0;
                        end
                    end 
                    3'b000: begin //addi
                        if (mem_rdata_I[31] == 0) begin
                            reg_rd_data = rs1_data + Immediate;
                        end
                        else begin
                            reg_rd_data = rs1_data - ((Immediate - 1) ^ 12'b111111111111);
                        end
                    end
                    default: begin 
                        reg_rd_data = 0;
                    end
                endcase
            end

            U_format: begin
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                ALUOp = 2'b00;
                Mode = 3;
                ALU_result = 0;
                reg_sw = 0;
                reg_lw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                Immediate = {mem_rdata_I[31:12], 12'b0};
                reg_rd_data = PC + Immediate;
            end

            lw: begin
                RegWrite = 1;
                MemRead = 1;
                MemWrite = 0;
                ALUOp = 2'b0;
                Mode = 3;
                ALU_result = 0;
                Immediate = mem_rdata_I[31:20];
                reg_sw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                reg_rd_data = mem_rdata_D;
                if (mem_rdata_I[31] == 0) begin
                    reg_lw = rs1_data + Immediate;
                end
                else begin
                    reg_lw = rs1_data - ((Immediate - 1) ^ 12'b111111111111);
                end
            end

            sw: begin
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 1;
                ALUOp = 2'b00;
                Mode = 3;
                ALU_result = 0;
                Immediate = {mem_rdata_I[31:25], mem_rdata_I[11:7]};
                reg_lw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                reg_rd_data = 0;
                if (mem_rdata_I[31] == 0) begin
                    reg_sw = rs1_data + Immediate;
                end
                else begin
                    reg_sw = rs1_data - ((Immediate - 1) ^ 12'b111111111111);
                end                
            end
            
            SB_format: begin
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                ALUOp = 2'b01;
                reg_rd_data = 0;
                reg_sw = 0;
                reg_lw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                if (mem_rdata_I[31] == 0) begin
                    Immediate = PC + {mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8],1'b0};    
                end
                else begin
                    Immediate = PC - (({mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8],1'b0} - 1) ^ 13'b1111111111111);
                end
                case (function3)
                    3'b000: begin
                        Mode = 0;
                        ALU_result = (rs1_data == rs2_data)? 1:0;
                    end
                    3'b101: begin
                        Mode = 1;
                        ALU_result = (rs1_data >= rs2_data)? 1:0;
                    end
                    default: begin
                        Mode = 0;
                        ALU_result = 0;
                    end
                endcase
            end

            JAL: begin
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                ALUOp = 2'b00;
                ALU_result = 0;
                Mode = 2;
                reg_sw = 0;
                reg_lw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                if (mem_rdata_I[31] == 0) begin
                    Immediate = PC + {mem_rdata_I[31], mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21], 1'b0};
                end
                else begin
                    Immediate = PC - (({mem_rdata_I[31], mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21], 1'b0} - 1) ^ 21'b111111111111111111111);
                end
                reg_rd_data = PC + 4;
            end

            JALR: begin
                RegWrite = 1;
                MemRead = 0;
                MemWrite = 0;
                ALUOp = 2'b00;
                Mode = 2;
                ALU_result = 0;
                reg_sw = 0;
                reg_lw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                if (mem_rdata_I[31] == 0) begin
                    Immediate = rs1_data + mem_rdata_I[31:20];
                end
                else begin
                    Immediate = rs1_data - ((mem_rdata_I[31:20] - 1) ^ 12'b111111111111);
                end
                reg_rd_data = PC + 4;
            end

            default: begin
                RegWrite = 0;
                MemRead = 0;
                MemWrite = 0;
                ALUOp = 2'b00;
                Mode = 3;
                ALU_result = 0;
                Immediate = 0;
                reg_sw = 0;
                reg_lw = 0;
                Valid = 0;
                mulDiv_Mode = 0;
                reg_rd_data = rd_data;
            end
        endcase
    end
    
    //==== Sequential Part ========================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00400000; // Do not modify this value!!!
        end
        else begin
            PC <= PC_nxt;
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0; // zero: hard-wired zero
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'h7fffeffc; // sp: stack pointer
                    32'd3: mem[i] <= 32'h10008000; // gp: global pointer
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2
    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input         mode; // mode: 0: mulu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [31:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter OUT  = 3'd3;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;
    wire [31:0] shreg_high;
    // Todo: Instatiate any primitives if needed
    
    // Todo 5: Wire assignments
    assign shreg_high = shreg[63:32];
    assign ready = (state == OUT);
    assign out = (ready)? shreg[31:0] : out;
    
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if(valid) begin
                    case(mode)
                        0: state_nxt = MUL;
                        1: state_nxt = DIV;
                    endcase
                end
                else begin 
                    state_nxt = IDLE;
                end
            end
            MUL : begin
                if(counter == 31) begin 
                    state_nxt = OUT;
                end
                else begin
                    state_nxt = state;
                end
            end
            DIV : begin 
                if(counter == 31) begin 
                    state_nxt = OUT;
                end
                else begin
                    state_nxt = state;
                end
            end
            OUT : state_nxt = IDLE;
            default : state_nxt = state;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        if(state == MUL || state == DIV) begin
            counter_nxt = counter + 1;
        end
        else begin
            counter_nxt = 0;
        end
    end
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MUL: begin
                alu_out = alu_in + shreg_high;
            end
            DIV: begin
                if(shreg_high > alu_in) begin
                    alu_out = shreg_high - alu_in;
                end
                else begin
                    alu_out = shreg_high;
                end
            end
            default: begin
                alu_out = alu_in;
            end
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: begin 
                if(valid) begin
                    if(state_nxt == DIV) begin
                        shreg_nxt = {31'b0,in_A,1'b0};
                    end
                    else begin
                        shreg_nxt = {32'b0,in_A};
                    end
                end
                else begin
                    shreg_nxt = shreg;
                end
            end
            MUL: begin
                shreg_nxt = shreg >> 1;
                if(shreg[0] == 1) begin
                    shreg_nxt[63:31] = alu_out;
                end                
            
            end
            DIV: begin
                shreg_nxt = shreg << 1;
                if(shreg_high > alu_in) begin
                    shreg_nxt[63:33] = alu_out[30:0];
                    shreg_nxt[0] = 1;
                end
                else begin
                    shreg_nxt[0] = 0;
                end
                
                if(counter == 31) begin
                    shreg_nxt[62:32] = shreg_nxt[63:33];
                    shreg_nxt[63] = 0;
                end 
            end
            default: shreg_nxt = shreg;
        endcase
    end
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= state_nxt;
            alu_in <= alu_in_nxt;
            shreg <= shreg_nxt;
            counter <= counter_nxt;
        end
    end

endmodule

module PC_control(PC,PC_nxt,immediate,alu_result,mode,halt);
    input [31:0] PC;
    output [31:0] PC_nxt;
    input [31:0] immediate;
    input [31:0] alu_result;
    input [1:0] mode; //beq = 0, bge = 1, jal = 2, else = 3
    input halt;

    reg is_branch;
    
    assign PC_nxt = (is_branch == 1)? immediate : 
                    (halt == 1)? PC : PC + 4;

    parameter beq = 0;
    parameter bge = 1;
    parameter jal = 2;
    parameter no_branch = 3;

    always @(*) begin
        case (mode)
            beq: begin
                is_branch = alu_result;
            end
            bge: begin
                is_branch = alu_result;
            end
            jal: begin
                is_branch = 1;
            end
            default: 
                is_branch = 0;
        endcase
    end
endmodule