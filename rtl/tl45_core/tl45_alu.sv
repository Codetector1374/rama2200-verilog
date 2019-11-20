`default_nettype none

module tl45_alu(
    i_clk, i_reset,
    i_pipe_stall, o_pipe_stall, // If i_pipe_stall is high, don't clock buffer.
    i_pipe_flush, o_pipe_flush, // Forward the flush, and clear the buffer
    // Buffer from previous stage
    i_pc,
    i_opcode,
    i_dr, i_skp_mode,
    i_sr1_val, i_sr2_val,
    i_target_address,
    // Operand Forward Outputs
    o_of_reg, o_of_val,
    // Current stage buffer
    o_dr, o_value, o_ld_newpc, o_br_pc
);
input wire i_clk, i_reset;
input wire i_pipe_stall, i_pipe_flush;
output wire o_pipe_flush, o_pipe_stall;

wire flush_previous_stage;
reg stall_previous_stage;
initial stall_previous_stage = 0;
// Flush Previous Stages when Upper stage stalls
assign o_pipe_stall = stall_previous_stage;
// Same with flush
assign o_pipe_flush = flush_previous_stage || i_pipe_flush;

// Input From previous stage
input wire [3:0] i_opcode;
input wire [3:0] i_dr;
input wire [31:0] i_sr1_val, i_sr2_val;
input wire [31:0] i_target_address, i_pc;
input wire i_skp_mode; // SKPEQ(0)  SKPLT(1)

// Operand Forward
output wire [3:0] o_of_reg;
output wire [31:0] o_of_val;

// Output to Next Stage
output reg [31:0] o_value;
output reg [3:0] o_dr;
output wire o_ld_newpc;
output reg [31:0] o_br_pc;

initial begin
    o_value = 0;
    o_dr = 0;
    o_br_pc = 0;
end

// Core ALU Logic
localparam OP_ADD = 4'b0000,
    OP_NAND = 4'b0001,
    OP_ADDI = 4'b0010,
    OP_LW = 4'b0011,
    OP_SW = 4'b0100,
    OP_GOTO = 4'b0101,
    OP_JALR = 4'b0110,
    OP_HALT = 4'b0111,
    OP_SKP = 4'b1000,
    OP_LEA = 4'b1001;

// Check if branch is executing
wire is_branch;
assign is_branch = i_opcode == OP_SKP; // Branch

// ALU Computaion Result
reg [31:0] alu_result;


// Main ALU
always @(*) begin
    case(i_opcode)
        OP_ADDI, OP_ADD: alu_result = i_sr1_val + i_sr2_val;
        OP_NAND: alu_result = ~(i_sr1_val & i_sr2_val);
        OP_JALR: alu_result = i_pc + 1;
        OP_LEA: alu_result = i_target_address;
        OP_SKP: alu_result = i_sr1_val - i_sr2_val;
        default: alu_result = 0;
    endcase
end

reg do_jump;
always @(*) begin
    if (i_opcode == OP_SKP)
        if (i_skp_mode == 1'b0) // SKPEQ
            do_jump = alu_result == 0;
        else // SKPLT
            do_jump = alu_result[31] == 1;
    else if (i_opcode == OP_JALR || i_opcode == OP_GOTO)
        do_jump = 1;
    else
        do_jump = 0;
end

// Target Address Computation
always @(*) begin
if (i_opcode == OP_JALR)
    o_br_pc = i_sr1_val;
else
    o_br_pc = i_target_address;
end

assign flush_previous_stage = do_jump; // Controlls JUMP
assign o_ld_newpc = do_jump; // when jump happens, loads new PC

wire should_alu_output;
assign should_alu_output = i_opcode == OP_ADD
                        || i_opcode == OP_NAND
                        || i_opcode == OP_ADDI
                        || i_opcode == OP_JALR
                        || i_opcode == OP_LEA;

always @(posedge i_clk) begin
    if (i_reset || i_pipe_flush) begin
        o_dr <= 0;
        o_value <= 0;
    end
    else if (should_alu_output) begin
        // ALU Logic
        o_dr <= i_dr;
        o_value <= alu_result;
    end else begin
        o_dr <= 0;
        o_value <= 0;
    end
end

assign o_of_reg = (should_alu_output) ? i_dr : 0;
assign o_of_val = (should_alu_output) ? alu_result : 0;

`ifdef FORMAL

reg f_past_valid;
initial f_past_valid = 0;

always @(posedge i_clk)
    f_past_valid <= 1;

always @(posedge i_clk) begin
if (f_past_valid)
    if ($past(o_pipe_flush)) begin // Assume correct previous stage flush behavior
        assume(i_dr == 0);
        assume(i_skp_mode == 0);
        assume(i_sr1_val == 0);
        assume(i_sr2_val == 0);
        assume(i_target_address == 0);
        assume(i_opcode == 0);
    end else if ($past(o_pipe_stall)) begin // Assume correct behavior from previous stage for stall
        assume(i_dr == $past(i_dr));
        assume(i_skp_mode == $past(i_skp_mode));
        assume(i_opcode == $past(i_opcode));
        assume(i_target_address == $past(i_target_address));
        assume(i_sr1_val == $past(i_sr1_val));
        assume(i_sr2_val == $past(i_sr2_val));
    end
end

always @(posedge i_clk) begin
if(f_past_valid)
    if ($past(is_branch)) begin
        assert(o_dr == 0);
        assert(o_value == 0);
    end
end

always @(*) begin
    if (do_jump && is_branch) begin
        assert(o_pipe_flush);
        assert(o_ld_newpc);
        assert(o_br_pc == i_sr1_val + i_target_offset);
    end
end

`endif

endmodule