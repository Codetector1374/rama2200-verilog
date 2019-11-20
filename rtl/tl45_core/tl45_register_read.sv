`default_nettype none
module tl45_register_read(
    i_clk, i_reset,
    i_pipe_stall, o_pipe_stall, // If i_pipe_stall is high, don't clock buffer.
    i_pipe_flush, o_pipe_flush, // Forward the flush, and clear the buffer
    // Buffer from previous stage
    i_pc,
    i_opcode, 
    i_skp_mode, // Register Immediate Mode
    i_dr, i_sr1, i_sr2, i_imm32,
    // DPRF Connections
    o_dprf_read_a1, o_dprf_read_a2,
    i_dprf_d1, i_dprf_d2,
    // Operand Forwarding Buses
    i_of1_reg, i_of1_data, // Operand Forwarding Bus #1 BEFORE ALU BUF
    i_of2_reg, i_of2_data, // Operand Forwarding Bus #2 AT ALU BUF
    // Output buffer from current stage
    o_pc,
    o_opcode,
    o_dr, o_skp_mode,
    o_sr1_val, o_sr2_val,
    o_target_address
);

// CPU Signals
input wire i_clk, i_reset;
input wire i_pipe_stall, i_pipe_flush;
output wire o_pipe_stall, o_pipe_flush;

// Previous stage input signal
input wire [3:0] i_opcode; // 4 bit opcode
input wire i_skp_mode; // 0 SKPEQ, 1 SKPLT
input wire [3:0] i_dr, i_sr1, i_sr2; // DR(FLAGS), SR1, SR2 address
input wire [31:0] i_imm32;
input wire [31:0] i_pc;

// DPRF Signals
output wire [3:0] o_dprf_read_a1, o_dprf_read_a2;
input wire [31:0] i_dprf_d1, i_dprf_d2;

input wire [3:0] i_of1_reg, i_of2_reg;
input wire [31:0] i_of1_data, i_of2_data;

// Stage Buffer
output reg [3:0] o_opcode;
output reg [3:0] o_dr;
output reg o_skp_mode;
output reg [31:0] o_sr1_val, o_sr2_val;
output reg [31:0] o_target_address, o_pc; // Target Jump Address Offset

// Flush Propgation
assign o_pipe_stall = i_pipe_stall;
assign o_pipe_flush = i_pipe_flush;

initial begin
    o_opcode = 0;
    o_dr = 0;
    o_sr1_val = 0;
    o_sr2_val = 0;
    o_pc = 0;
    o_target_address = 0;
    o_skp_mode = 0;
end


// DPRF Reading Stuff
assign o_dprf_read_a1 = i_sr1;
assign o_dprf_read_a2 = i_sr2;

always @(posedge i_clk) begin
    if (i_reset || i_pipe_flush) begin
        // Clear Buffer
        o_opcode <= 4'hF;
        o_dr <= 0;
        o_sr1_val <= 0;
        o_sr2_val <= 0;
        o_pc <= 0;
        o_target_address <= 0;
        o_skp_mode <= 0;
    end
    else if (!i_pipe_stall) begin
        o_target_address <= i_imm32;
        o_opcode <= i_opcode;
        o_pc <= i_pc;
        o_dr <= i_dr;
        o_skp_mode <= i_skp_mode;
        // SR1 Operand Forwarding Checking
        // LOGIC:
        // -> 1) if OpFwdBus1 has the register value, load that
        //      it comes from the ALU BEFORE ALU Buffer
        // -> 2) if the OpFwdBus2 has the register value, load
        //      that. It comes from ALU AFTER ALU Buffer
        // -> 3) Otherwise trust the value coming from the DPRF
        if ((i_sr1 != 4'h0) && (i_of1_reg == i_sr1)) begin
            o_sr1_val <= i_of1_data;
        end else if ((i_sr1 != 4'h0) && (i_sr1 == i_of2_reg)) begin
            o_sr1_val <= i_of2_data;
        end
        else begin
            o_sr1_val <= i_dprf_d1;
        end

        // SR2 Operand Forwarding Checking
        // IMM Priority
        if (i_opcode == 4'b0010) // ADDI
            o_sr2_val <= i_imm32;
        else if ((i_sr2 != 4'h0) && (i_sr2 == i_of1_reg)) begin // Operand Fwd from BUS#1
            o_sr2_val <= i_of1_data;
        end else if ((i_sr2 != 4'h0) && (i_sr2 == i_of2_reg)) begin // Operand Fwd from BUS#2
            o_sr2_val <= i_of2_data;
        end else begin // Trust the value from the dprf
            o_sr2_val <= i_dprf_d2;
        end
    end
end

endmodule
