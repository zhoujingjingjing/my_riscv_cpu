module regfile(
    input  wire        clk,
    input  wire        reset,
    // READ PORT 1
    input  wire [ 4:0] raddr1,
    output wire [31:0] rdata1,
    // READ PORT 2
    input  wire [ 4:0] raddr2,
    output wire [31:0] rdata2,
    // WRITE PORT
    input  wire        we,       //write enable, HIGH valid
    input  wire [ 4:0] waddr,
    input  wire [31:0] wdata,
    output wire [1023:0] debug_gpr_flat
);
reg [31:0] rf[31:0];
integer i;

//WRITE
always @(posedge clk) begin
    if (reset) begin
        for (i = 1; i < 32; i = i + 1) rf[i] <= 32'b0;
    end else if (we && waddr != 5'b0) begin
        rf[waddr] <= wdata;
    end
end

// bit [32*n +: 32] 恒等于 xn；x0不依赖存储单元，始终为0。
genvar g;
generate
    for (g = 0; g < 32; g = g + 1) begin : GEN_DEBUG_GPR
        if (g == 0) begin : GEN_X0
            assign debug_gpr_flat[31:0] = 32'b0;
        end else begin : GEN_XN
            assign debug_gpr_flat[g*32 +: 32] = rf[g];
        end
    end
endgenerate

//READ OUT 1
assign rdata1 = (raddr1==5'b0) ? 32'b0 : rf[raddr1];

//READ OUT 2
assign rdata2 = (raddr2==5'b0) ? 32'b0 : rf[raddr2];

endmodule
