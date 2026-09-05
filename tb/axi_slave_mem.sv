// =============================================================================
// axi_slave_mem.sv
// WaveGuard 3x10 AXI Interconnect Verification
//
// AXI4 compliant slave memory model.
// - Accepts AW / W / AR channels from the interconnect master ports
// - Returns B / R responses with programmable latency
// - Internal 64 KB byte-addressable SRAM (word-aligned 32-bit)
// - Exposes a backdoor read/write task for testbench checking
// - Reports which slave index it is via SLAVE_ID parameter
// =============================================================================

`timescale 1ns/1ps

module axi_slave_mem #(
    parameter DATA_WIDTH   = 32,
    parameter ADDR_WIDTH   = 32,
    parameter STRB_WIDTH   = (DATA_WIDTH/8),
    parameter ID_WIDTH     = 8,
    parameter MEM_SIZE     = 65536,      // bytes
    parameter RESP_LATENCY = 1,          // cycles before asserting awready/arready
    parameter SLAVE_ID     = 0           // which slave slot (for display only)
) (
    input  logic                    clk,
    input  logic                    rst,

    // AW channel
    input  logic [ID_WIDTH-1:0]     axi_awid,
    input  logic [ADDR_WIDTH-1:0]   axi_awaddr,
    input  logic [7:0]              axi_awlen,
    input  logic [2:0]              axi_awsize,
    input  logic [1:0]              axi_awburst,
    input  logic                    axi_awlock,
    input  logic [3:0]              axi_awcache,
    input  logic [2:0]              axi_awprot,
    input  logic [3:0]              axi_awqos,
    input  logic [3:0]              axi_awregion,
    input  logic                    axi_awvalid,
    output logic                    axi_awready,

    // W channel
    input  logic [DATA_WIDTH-1:0]   axi_wdata,
    input  logic [STRB_WIDTH-1:0]   axi_wstrb,
    input  logic                    axi_wlast,
    input  logic                    axi_wvalid,
    output logic                    axi_wready,
    output logic [ID_WIDTH-1:0]     axi_bid,
    output logic [1:0]              axi_bresp,
    output logic                    axi_bvalid,
    input  logic                    axi_bready,
    // AR channel
    input  logic [ID_WIDTH-1:0]     axi_arid,
    input  logic [ADDR_WIDTH-1:0]   axi_araddr,
    input  logic [7:0]              axi_arlen,
    input  logic [2:0]              axi_arsize,
    input  logic [1:0]              axi_arburst,
    input  logic                    axi_arlock,
    input  logic [3:0]              axi_arcache,
    input  logic [2:0]              axi_arprot,
    input  logic [3:0]              axi_arqos,
    input  logic [3:0]              axi_arregion,
    input  logic                    axi_arvalid,
    output logic                    axi_arready,

    // R channel
    output logic [ID_WIDTH-1:0]     axi_rid,
    output logic [DATA_WIDTH-1:0]   axi_rdata,
    output logic [1:0]              axi_rresp,
    output logic                    axi_rlast,
    output logic                    axi_rvalid,
    input  logic                    axi_rready
);

    // -------------------------------------------------------------------------
    // Internal memory
    // -------------------------------------------------------------------------
    localparam WORDS = MEM_SIZE / (DATA_WIDTH/8);
    localparam WORD_BITS = $clog2(DATA_WIDTH/8);
   logic [DATA_WIDTH-1:0] mem [0:WORDS-1];

   integer b;
   integer i;
   integer widx;
   integer word_idx;
   logic [ADDR_WIDTH-1:0] beat_addr;
   // -------------------------------------------------------------------------
    // Write path state machine
    //   States: AW_IDLE → AW_ACCEPT → W_DATA → B_RESP
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        WR_IDLE     = 2'd0,
        WR_ACCEPT   = 2'd1,
        WR_DATA     = 2'd2,
        WR_BRESP    = 2'd3
    } wr_state_t;

    wr_state_t wr_state;

    logic [ID_WIDTH-1:0]   wr_id;
    logic [ADDR_WIDTH-1:0] wr_addr;
    logic [7:0]            wr_len;
    logic [2:0]            wr_size;
    logic [1:0]            wr_burst;
    logic [7:0]            wr_beat;

    // AW handshake
    always @(posedge clk) begin
        if (rst) begin
            wr_state    <= WR_IDLE;
            axi_awready <= 1'b0;
            axi_wready  <= 1'b0;
            axi_bvalid  <= 1'b0;
            axi_bid     <= '0;
            axi_bresp   <= 2'b00;
            wr_beat     <= '0;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    axi_awready <= 1'b1;
                    axi_wready  <= 1'b0;
                    if (axi_awvalid && axi_awready) begin
                        // accept the AW beat
                        axi_awready <= 1'b1;
                        wr_id    <= axi_awid;
                        wr_addr  <= axi_awaddr;
                        wr_len   <= axi_awlen;
                        wr_size  <= axi_awsize;
                        wr_burst <= axi_awburst;
                        wr_beat  <= 8'd0;
                        wr_state <= WR_ACCEPT;
                    end
                end

                WR_ACCEPT: begin
                    // awready was high last cycle; lower it; open W
                    axi_awready <= 1'b0;
                    axi_wready  <= 1'b1;
                    wr_state    <= WR_DATA;
                end

                WR_DATA: begin
                    if (axi_wvalid && axi_wready) begin
                        // write data beat to memory
                        logic [ADDR_WIDTH-1:0] beat_addr;
                        // INCR burst: addr increments by transfer size each beat
                        beat_addr = wr_addr + (wr_beat << wr_size);

               for (b = 0; b < STRB_WIDTH; b = b + 1) begin
    		       if (axi_wstrb[b]) begin
                      logic [ADDR_WIDTH-1:0] byte_addr;
                      byte_addr = beat_addr + b;
                      word_idx = byte_addr[ADDR_WIDTH-1:WORD_BITS] % WORDS;

                      mem[word_idx][b*8 +: 8] <= axi_wdata[b*8 +: 8];
                   end
               end
                        wr_beat <= wr_beat + 1;

                        if (axi_wlast) begin
                            axi_wready <= 1'b0;
                            axi_bvalid <= 1'b1;
                            axi_bid    <= wr_id;
                            axi_bresp  <= 2'b00; // OKAY
                            wr_state   <= WR_BRESP;
                        end
                    end
                end

                WR_BRESP: begin
                    if (axi_bvalid && axi_bready) begin
                        axi_bvalid <= 1'b0;
                        wr_state   <= WR_IDLE;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Read path state machine
    //   States: AR_IDLE → AR_ACCEPT → R_DATA
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        RD_IDLE   = 2'd0,
        RD_ACCEPT = 2'd1,
        RD_DATA   = 2'd2,
        RD_WAIT   = 2'd3
    } rd_state_t;

    rd_state_t rd_state;

    logic [ID_WIDTH-1:0]   rd_id;
    logic [ADDR_WIDTH-1:0] rd_addr;
    logic [7:0]            rd_len;
    logic [2:0]            rd_size;
    logic [1:0]            rd_burst;
    logic [7:0]            rd_beat;

    always @(posedge clk) begin
        if (rst) begin
            rd_state    <= RD_IDLE;
            axi_arready <= 1'b0;
            axi_rvalid  <= 1'b0;
            axi_rid     <= '0;
            axi_rdata   <= '0;
            axi_rresp   <= 2'b00;
            axi_rlast   <= 1'b0;
            rd_beat     <= '0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    axi_arready <= 1'b1;
                    if (axi_arvalid && axi_arready) begin
                        axi_arready <= 1'b1;
                        rd_id    <= axi_arid;
                        rd_addr  <= axi_araddr;
                        rd_len   <= axi_arlen;
                        rd_size  <= axi_arsize;
                        rd_burst <= axi_arburst;
                        rd_beat  <= 8'd0;
                        rd_state <= RD_ACCEPT;
                    end
                end

                RD_ACCEPT: begin
                    axi_arready <= 1'b0;

                    beat_addr = rd_addr + (rd_beat << rd_size);
                    word_idx  = beat_addr[ADDR_WIDTH-1:WORD_BITS] % WORDS;

                    axi_rid    <= rd_id;
                    axi_rdata  <= mem[word_idx];
                    axi_rresp  <= 2'b00;
                    axi_rlast  <= (rd_beat == rd_len);
                    axi_rvalid <= 1'b1;

                    rd_state <= RD_DATA;
                end

                RD_DATA: begin
                    if (axi_rvalid && axi_rready) begin
                        if (axi_rlast) begin
                            axi_rvalid <= 1'b0;
                            rd_state   <= RD_IDLE;
                        end else begin
                            rd_beat <= rd_beat + 1;
                            beat_addr = rd_addr + ((rd_beat + 1) << rd_size);
                            word_idx  = beat_addr[ADDR_WIDTH-1:WORD_BITS] % WORDS;

                            axi_rid    <= rd_id;
                            axi_rdata  <= mem[word_idx];
                            axi_rresp  <= 2'b00;
                            axi_rlast  <= ((rd_beat + 1) == rd_len);
                            axi_rvalid <= 1'b1;
                        end
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Backdoor tasks for testbench
    // -------------------------------------------------------------------------
    // Write a word directly into memory (no AXI handshake)
    task backdoor_write(input logic [ADDR_WIDTH-1:0] addr,
                        input logic [DATA_WIDTH-1:0] data);
        integer widx;
widx = addr[ADDR_WIDTH-1:WORD_BITS] % WORDS;
        mem[widx] = data;
    endtask

    // Read a word directly from memory
    task backdoor_read(input  logic [ADDR_WIDTH-1:0] addr,
                       output logic [DATA_WIDTH-1:0] data);
        integer widx;
widx = addr[ADDR_WIDTH-1:WORD_BITS] % WORDS;        data = mem[widx];
    endtask

    // Initialise all memory to zero
    task mem_clear();
        for (i = 0; i < WORDS; i = i + 1)
    		mem[i] = '0;
    endtask

    endmodule

