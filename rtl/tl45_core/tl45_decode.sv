`default_nettype none

module tl45_decode(
    i_clk, i_reset,
    i_pipe_stall, o_pipe_stall,
    i_pipe_flush, o_pipe_flush,

    // Buffer In
    i_buf_pc, i_buf_inst,

    // Buffer Out
    o_buf_pc,
    o_buf_opcode, o_buf_skp_mode,
    o_buf_dr, o_buf_sr1, o_buf_sr2,
    o_buf_imm
);

input wire i_clk, i_reset;
input wire i_pipe_stall;
output reg o_pipe_stall;
input wire i_pipe_flush;
output reg o_pipe_flush;
initial o_pipe_stall = 0;

input wire [31:0] i_buf_pc;
input wire [31:0] i_buf_inst;

output reg [31:0] o_buf_pc;
output reg [3:0] o_buf_opcode;
output reg o_buf_skp_mode;
output reg [3:0] o_buf_dr, o_buf_sr1, o_buf_sr2;
output reg [31:0] o_buf_imm;


// Pipeline flush stalls propgate
assign o_pipe_flush = i_pipe_flush;
assign o_pipe_stall = i_pipe_stall;

wire current_stage_flush;
assign current_stage_flush = i_pipe_flush;

initial begin
    o_buf_pc = 0;
    o_buf_opcode = 4'hF;
    o_buf_skp_mode = 0;
    o_buf_dr = 0;
    o_buf_sr1 = 0;
    o_buf_sr2 = 0;
    o_buf_imm = 0;
end

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

// Always decode
wire [3:0] opcode;
assign opcode = i_buf_inst[31:28];
reg [3:0] trans_op;
always @(*) begin
    // OPCODE DECODE
    case(i_buf_inst[31:28])
        OP_HALT: trans_op = OP_GOTO;
        default: trans_op = i_buf_inst[31:28];
    endcase
end
reg [3:0] dr, sr1, sr2;
reg [31:0] immvalue;

function [31:0] computePcOffset;
input [31:0] pc;
input [19:0] offset;
assign computePcOffset = {{12{offset[19]}}, offset[19:0]} + pc + 1;
endfunction

always @(*) begin
    case(opcode)
        OP_ADD,
        OP_NAND: begin
            dr = i_buf_inst[27:24];
            sr1 = i_buf_inst[23:20];
            sr2 = i_buf_inst[3:0];
            immvalue = 0;
        end
        OP_ADDI: begin
            dr = i_buf_inst[27:24];
            sr1 = i_buf_inst[23:20];
            sr2 = 0;
            immvalue = {{12{i_buf_inst[19]}}, i_buf_inst[19:0]}; // SEXT IMM20
        end
        OP_LW: begin
            dr = i_buf_inst[27:24];
            sr1 = i_buf_inst[23:20]; // BaseR
            sr2 = 0;
            immvalue = {{12{i_buf_inst[19]}}, i_buf_inst[19:0]}; // SEXT IMM20
        end
        OP_SW: begin
            dr = 0;
            sr1 = i_buf_inst[23:20]; // BaseR
            sr2 = i_buf_inst[27:24]; // Source
            immvalue = {{12{i_buf_inst[19]}}, i_buf_inst[19:0]}; // SEXT IMM20
        end
        OP_GOTO: begin
            dr = 0;
            sr1 = 0;
            sr2 = 0;
            immvalue = computePcOffset(i_buf_pc, i_buf_inst[19:0]); // Target Address
        end
        OP_JALR: begin
            dr = i_buf_inst[27:24];
            sr1 = i_buf_inst[23:20];
            sr2 = 0;
            immvalue = 0;
        end
        OP_SKP: begin
            dr = 0;
            sr1 = i_buf_inst[23:20];
            sr2 = i_buf_inst[3:0];
            immvalue = 0;
        end
        OP_LEA: begin
            dr = i_buf_inst[27:24];
            sr1 = 0;
            sr2 = 0;
            immvalue = computePcOffset(i_buf_pc, i_buf_inst[19:0]); // Target Address
        end
        OP_HALT: begin
            dr = 0;
            sr1 = 0;
            sr2 = 0;
            immvalue = i_buf_pc;
        end
        default: begin
            dr = 0;
            sr1 = 0;
            sr2 = 0;
            immvalue = 0;
        end
    endcase
end

always @(posedge i_clk) begin
    if (i_reset || current_stage_flush) begin
        o_buf_pc <= 0;
        o_buf_opcode <= 4'hF;
        o_buf_skp_mode <= 0;
        o_buf_dr <= 0;
        o_buf_sr1 <= 0;
        o_buf_sr2 <= 0;
        o_buf_imm <= 0;
    end
    else if (!i_pipe_stall) begin
        o_buf_pc <= i_buf_pc;
        o_buf_opcode <= trans_op;
        o_buf_skp_mode <= i_buf_inst[24];
        o_buf_dr <= dr;
        o_buf_sr1 <= sr1;
        o_buf_sr2 <= sr2;
        o_buf_imm <= immvalue;
    end
end

endmodule







