`default_nettype none

module tl45_memory(
    i_clk, i_reset,
    i_pipe_stall, o_pipe_stall,
    i_pipe_flush, o_pipe_flush,

    // Wishbone
    o_wb_cyc, o_wb_stb, o_wb_we,
    o_wb_addr, o_wb_data, o_wb_sel,
    i_wb_ack, i_wb_stall, i_wb_err,
    i_wb_data,

    // Buffer from previous stage
    i_pc,
    i_buf_opcode,
    i_dr,
    i_sr1_val, i_sr2_val,
    i_target_address,

    // Forwarding
    o_fwd_dr, o_fwd_val,

    // Buffer Out
    o_buf_dr, o_buf_val
);

input wire i_clk, i_reset;
input wire i_pipe_stall;
output reg o_pipe_stall;
input wire i_pipe_flush;
output reg o_pipe_flush;

// Wishbone 
output reg o_wb_cyc, o_wb_stb, o_wb_we;
output reg [29:0] o_wb_addr;
output reg [31:0] o_wb_data;
output reg [3:0] o_wb_sel;
initial begin
    o_wb_cyc = 0;
    o_wb_stb = 0;
    o_wb_we = 0;
    o_wb_data = 0;
    o_wb_addr = 0;
end

input wire i_wb_ack, i_wb_stall, i_wb_err;
input wire [31:0] i_wb_data;

// Input From previous stage
input wire [3:0] i_buf_opcode;
input wire [3:0] i_dr;
input wire [31:0] i_sr1_val, i_sr2_val;
input wire [31:0] i_target_address, i_pc;

// Forwarding
output reg [3:0] o_fwd_dr;
output reg [31:0] o_fwd_val;
initial begin
    o_fwd_dr = 0;
    o_fwd_val = 0;
end

// Buffer Out
output reg [3:0] o_buf_dr;
output reg [31:0] o_buf_val;
initial begin
    o_buf_dr = 0;
    o_buf_val = 0;
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

// Internal

// Small inconsistencies between LW, SW, IN, and OUT make this a little
// annoying. Here we generate a bunch of combinational logic so that the state
// machine is simpler.
//
// Buffer Layout:
//  LW: dr <- MEM[sr1+imm]
//  SW: MEM[sr1+imm] <- sr2
//  IN: dr <- IO[imm]
// OUT: IO[imm] <- sr1
//
// However IO is mapped onto the memory bus so mem_addr will hold the mapped
// address for IO. The correct write value will be resolved into wr_val.
//
// IO is mapped into the highest 16 bits of memory however the memory bus is
// only 30 bits wide. To reconcile this, the top 2 bits of the IO imm are
// ignored. 16 + 14 = 30 bits. mem_addr is 32 bits wide. The bottom two bits
// select within a 32 bit word for instruction where this is supported. (none
// right now). When sent on the wishbone bus, only the top 30 bits of mem_addr
// are sent.

wire start_tx;
assign start_tx = (i_buf_opcode == OP_SW || 
                   i_buf_opcode == OP_LW);

wire is_io;
assign is_io =    0;

wire is_write;
assign is_write = (i_buf_opcode == OP_SW);

reg [31:0] mem_addr;
always @(*)
        mem_addr = i_sr1_val + i_target_address;

wire [31:0] wr_val;
assign wr_val = i_sr2_val;

reg [3:0] wb_sel_val;
always @(*)
    wb_sel_val = 4'b1111;

reg [31:0] write_data;
always @(*)
        write_data = wr_val;

reg [31:0] in_data;
always @(*)
    in_data = i_wb_data;

localparam
    IDLE = 0,
    READ_STROBE = 1,
    READ_WAIT_ACK = 2,
    READ_STALLED_OUT = 3,
    READ_OUT = 4,
    WRITE_STROBE = 5,
    WRITE_WAIT_ACK = 6,
    LAST_STATE = 7;

reg [3:0] current_state;
initial current_state = IDLE;

// wishbone combinational control signals
assign o_wb_stb = (current_state == READ_STROBE) || (current_state == WRITE_STROBE);
assign o_wb_we  = (current_state == WRITE_STROBE);
assign o_wb_cyc = (current_state != IDLE && current_state != LAST_STATE);

reg internal_stall;
initial internal_stall = 0;
assign o_pipe_stall = i_pipe_stall || internal_stall;

// internal state machine control signals
wire state_strobe, state_wait_ack;
assign state_strobe = (current_state == READ_STROBE) || (current_state == WRITE_STROBE);
assign state_wait_ack = (current_state == READ_WAIT_ACK) || (current_state == WRITE_WAIT_ACK);

always @(*)
    case (current_state)
        IDLE: internal_stall = start_tx;
        READ_STROBE,
        WRITE_STROBE: internal_stall = 1;
        READ_WAIT_ACK,
        WRITE_WAIT_ACK: internal_stall = 1;
        READ_STALLED_OUT: internal_stall = 1;
        READ_OUT: internal_stall = 0;
        default: internal_stall = 1;
    endcase

reg [31:0] temp_read;

always @(*)
    case (current_state)
        READ_STALLED_OUT: begin
            o_fwd_dr = !is_write ? i_dr : 0;
            o_fwd_val = !is_write ? temp_read : 0;
        end
        READ_WAIT_ACK: begin
            o_fwd_dr = !i_pipe_stall ? i_dr : 0;
            o_fwd_val = !i_pipe_stall ? (i_wb_err ? 32'h13371337 : i_wb_data) : 0;
        end
        default: begin
            o_fwd_dr = 0;
            o_fwd_val = 0;
        end
    endcase

always @(posedge i_clk) begin
    if (i_reset || i_pipe_flush) begin // TODO i_pipe_flush not handled properly
        current_state <= IDLE;

        o_wb_addr <= 0;
        o_wb_data <= 0;
        o_wb_sel <= 0;

        o_buf_dr <= 0;
        o_buf_val <= 0;
    end
    else if (i_pipe_stall) begin
    end
    else if ((current_state == IDLE) && start_tx) begin
        current_state <= is_write ? WRITE_STROBE : READ_STROBE;
        o_wb_addr <= mem_addr[29:0];
        o_wb_sel <= wb_sel_val;

        if (is_write)
            o_wb_data <= write_data;
    end
    else if (state_strobe && !i_wb_stall) begin
        current_state <= current_state == READ_STROBE ? READ_WAIT_ACK : WRITE_WAIT_ACK;
        o_wb_addr <= 0;

        o_wb_data <= 0;
    end
    // TODO probably not correct when wb response is on the same clock as the
    // request.
    else if (state_wait_ack && i_wb_err) begin
        // for now we'll just squash bus error as a special read value.
        // for write, whatever.
        current_state <= current_state == READ_WAIT_ACK ? (i_pipe_stall ? READ_STALLED_OUT : READ_OUT) : IDLE;

        if (!is_write) begin
            if (i_pipe_stall)
                temp_read <= 32'h13371337;
            else begin
                o_buf_dr <= i_dr;
                o_buf_val <= 32'h13371337;
            end
        end
    end
    else if (state_wait_ack && i_wb_ack && !i_wb_err) begin

        if (current_state == READ_WAIT_ACK)
            current_state <= i_pipe_stall ? READ_STALLED_OUT : READ_OUT;
        else
            current_state <= READ_OUT;

        o_wb_sel <= 0;

        if (!is_write) begin
            if (i_pipe_stall)
                temp_read <= in_data;
            else begin
                o_buf_dr <= i_dr;
                o_buf_val <= in_data;
            end
        end
    end
    else if (current_state == READ_STALLED_OUT && !i_pipe_stall) begin
        current_state <= READ_OUT;
        o_buf_dr <= i_dr;
        o_buf_val <= temp_read;
        temp_read <= 0;
    end
    else if (current_state == READ_OUT) begin
        current_state <= IDLE;
        o_buf_dr <= 0;
        o_buf_val <= 0;
        o_pipe_flush <= 0;
    end
end


`ifdef FORMAL

reg f_past_valid;
initial f_past_valid = 0;
always @(posedge i_clk)
    f_past_valid <= 1;
always @(*)
    assert(current_state < LAST_STATE);

initial assume(i_reset); // start in reset
initial	assume(!i_wb_ack);
initial	assume(!i_wb_err);

always @(posedge i_clk) begin
    if ($past(i_reset)) begin
        assert(current_state == IDLE); // We Can Reset
    end
end
// BUS Verify
    wire [3:0] f_nreqs, f_nacks, f_outstanding;
    fwb_master #(
            .AW(30),
            .DW(32),
            .F_MAX_STALL(0),
			.F_MAX_ACK_DELAY(0),
			.F_OPT_RMW_BUS_OPTION(0),
			.F_OPT_DISCONTINUOUS(1))
		f_wbm(i_clk, i_reset,
			o_wb_cyc, o_wb_stb, o_wb_we, o_wb_addr, o_wb_data, o_wb_sel,
			i_wb_ack, i_wb_stall, 32'h0, i_wb_err,
			f_nreqs, f_nacks, f_outstanding);

`endif


endmodule


