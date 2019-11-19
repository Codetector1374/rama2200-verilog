`default_nettype none

module tl45_decode(
    i_clk, i_reset,
    i_pipe_stall, o_pipe_stall,
    i_pipe_flush, o_pipe_flush,

    // Buffer In
    i_buf_pc, i_buf_inst,

    // Buffer Out
    o_buf_pc,
    o_buf_opcode,
    o_buf_dr, o_buf_sr1, o_buf_sr2
);

input wire i_clk, i_reset;
input wire i_pipe_stall;
output reg o_pipe_stall;
input wire i_pipe_flush;
output reg o_pipe_flush;
initial o_pipe_stall = 0;

input wire [31:0] i_buf_pc, i_buf_inst;

output reg [31:0] o_buf_pc;
output reg [3:0] o_buf_opcode;
output reg [3:0] o_buf_dr;
output reg [31:0] o_buf_sr1, o_buf_sr2;

initial begin
    o_buf_pc = 0;
    o_buf_opcode = 0;
    o_buf_dr = 0;
    o_buf_sr1 = 0;
    o_buf_sr2 = 0;
end

wire [3:0] opcode;
assign opcode = i_buf_inst[31:28];
reg [3:0] r_dr;
reg [31:0] r_sr1, r_sr2;

always @(*) begin
    case(opcode)

        
        default:
    endcase
end

always @(posedge i_clk) begin
    if (i_reset) begin
        o_buf_pc <= 0;
        o_buf_opcode <= 4'hF; // 1111 is NOP
        o_buf_dr <= 0;
        o_buf_sr1 <= 0;
        o_buf_sr2 <= 0;
    end
end

endmodule : tl45_decode







