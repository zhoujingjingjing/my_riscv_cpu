// [阶段3修改] Xilinx FPGA专用4读2写通用寄存器堆。
// 结构参考biRISC-V biriscv_regfile.v：
// 结构参考biRISC-V biriscv_regfile.v（提交6af9c4be5a08，Apache-2.0）：
// 两个逻辑写银行分别保存write0/write1产生的值，owner记录每个架构寄存器
// 的最新值在哪个银行；每个读取槽同时读取两个银行，再由owner作最后选择。
// 文件职责：实现双发射需要的四读两写架构寄存器堆和调试快照。
// 上游：ISSUE提供四个读地址，WB提供两个按年龄排序的写端口。
// 下游：ISSUE获得四个读数据，顶层调试接口获得32个架构寄存器的平面快照。
module regfile(
    input  wire        clk,
    input  wire        reset,

    input  wire [4:0]  raddr0,
    output wire [31:0] rdata0,
    input  wire [4:0]  raddr1,
    output wire [31:0] rdata1,
    input  wire [4:0]  raddr2,
    output wire [31:0] rdata2,
    input  wire [4:0]  raddr3,
    output wire [31:0] rdata3,

    input  wire        we0,
    input  wire [4:0]  waddr0,
    input  wire [31:0] wdata0,
    input  wire        we1,
    input  wire [4:0]  waddr1,
    input  wire [31:0] wdata1,

    output wire [1023:0] debug_gpr_flat
);
    //============================================================
    // logical write ports and four asynchronous read ports
    //============================================================

    // 白话信号导读：
    // wr0/wr1是过滤掉x0后的真实写使能。RISC-V的x0永远是0，所以写x0就像写到空气里，不能进RAM。
    // r0/r1/r2/r3对应四个读口，通常给两条指令的rs1/rs2使用：slot0读r0/r1，slot1读r2/r3。
    // bank0/bank1不是两个架构寄存器堆，而是两个“最近由哪个写口写过”的数据仓库。
    // r*_bank0表示这个读口从写口0仓库读出的候选值；r*_bank1表示从写口1仓库读出的候选值。
    // 后面的owner会像门牌一样告诉每个架构寄存器，该相信bank0还是bank1。
    wire wr0 = we0 && (waddr0 != 5'b0);
    wire wr1 = we1 && (waddr1 != 5'b0);

    wire [31:0] r0_bank0, r1_bank0, r2_bank0, r3_bank0;
    wire [31:0] r0_bank1, r1_bank1, r2_bank1, r3_bank1;

    // slot0的两个读口各查看write0、write1两个逻辑银行。
    xilinx_2r1w u_read0_bank0 (
        .clk(clk), .we(wr0), .waddr(waddr0), .wdata(wdata0),
        .raddr0(raddr0), .raddr1(raddr1),
        .rdata0(r0_bank0), .rdata1(r1_bank0)
    );
    xilinx_2r1w u_read0_bank1 (
        .clk(clk), .we(wr1), .waddr(waddr1), .wdata(wdata1),
        .raddr0(raddr0), .raddr1(raddr1),
        .rdata0(r0_bank1), .rdata1(r1_bank1)
    );

    // slot1的两个读口复制读取硬件，但内容由同样的两个写口保持一致。
    xilinx_2r1w u_read1_bank0 (
        .clk(clk), .we(wr0), .waddr(waddr0), .wdata(wdata0),
        .raddr0(raddr2), .raddr1(raddr3),
        .rdata0(r2_bank0), .rdata1(r3_bank0)
    );
    xilinx_2r1w u_read1_bank1 (
        .clk(clk), .we(wr1), .waddr(waddr1), .wdata(wdata1),
        .raddr0(raddr2), .raddr1(raddr3),
        .rdata0(r2_bank1), .rdata1(r3_bank1)
    );

    //============================================================
    // owner and valid state
    //============================================================

    // owner[x]=0表示x的最新值由写口0银行保存，=1表示由写口1银行保存。
    // valid避免依赖LUTRAM复位清零；复位后尚未写过的架构寄存器直接读0。
    // owner可以理解成32张小纸条，每张纸条贴在一个架构寄存器名下，写着“最新版本在哪个银行”。
    // valid是另外32张纸条，说明这个寄存器复位后有没有被真正写过；没写过就不信LUTRAM的初值，直接读0。
    // 同拍双写同一个rd时，slot1更年轻，所以wr1后写owner，最终让读口看到slot1那份最新值。
    reg [31:0] owner;
    reg [31:0] valid;
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            owner <= 32'b0;
            valid <= 32'b0;
        end else begin
            if (wr0) begin
                owner[waddr0] <= 1'b0;
                valid[waddr0] <= 1'b1;
            end
            // slot1较年轻，所以同拍写同一个rd时由写口1覆盖owner并最终生效。
            if (wr1) begin
                owner[waddr1] <= 1'b1;
                valid[waddr1] <= 1'b1;
            end
            owner[0] <= 1'b0;
            valid[0] <= 1'b0;
        end
    end

    // raw0~raw3是按owner挑出的候选读数，还没有处理x0和valid。
    // rdata0~rdata3是最终给ISSUE看的读数：读x0或读到“复位后没写过”的寄存器，都统一返回0。
    wire [31:0] raw0 = owner[raddr0] ? r0_bank1 : r0_bank0;
    wire [31:0] raw1 = owner[raddr1] ? r1_bank1 : r1_bank0;
    wire [31:0] raw2 = owner[raddr2] ? r2_bank1 : r2_bank0;
    wire [31:0] raw3 = owner[raddr3] ? r3_bank1 : r3_bank0;
    assign rdata0 = (raddr0 == 0 || !valid[raddr0]) ? 32'b0 : raw0;
    assign rdata1 = (raddr1 == 0 || !valid[raddr1]) ? 32'b0 : raw1;
    assign rdata2 = (raddr2 == 0 || !valid[raddr2]) ? 32'b0 : raw2;
    assign rdata3 = (raddr3 == 0 || !valid[raddr3]) ? 32'b0 : raw3;

    //============================================================
    // architectural debug snapshot
    //============================================================

    // 调试快照不进入执行数据路径。它只服务NEMU退休级逐条对拍和波形观察，
    // 因此单独维护不会把1024位调试扇出串到ISSUE关键路径中。
    // debug_rf是一份“给验收老师看的成绩册”，不参与CPU真正算数，只记录退休后每个寄存器应是什么。
    // debug_gpr_flat把32个32位寄存器摊平成1024位总线，方便顶层一次性交给NEMU和测试框架。
    // GEN_DEBUG_GPR里x0固定接0，其余寄存器按g*32的位置平铺，顺序和退休接口约定一致。
    reg [31:0] debug_rf [31:0];
    always @(posedge clk) begin
        if (reset) begin
            for (i = 1; i < 32; i = i + 1)
                debug_rf[i] <= 32'b0;
        end else begin
            if (wr0) debug_rf[waddr0] <= wdata0;
            if (wr1) debug_rf[waddr1] <= wdata1;
        end
    end

    genvar g;
    generate
        for (g = 0; g < 32; g = g + 1) begin : GEN_DEBUG_GPR
            if (g == 0) begin : GEN_X0
                assign debug_gpr_flat[31:0] = 32'b0;
            end else begin : GEN_XN
                assign debug_gpr_flat[g*32 +: 32] = debug_rf[g];
            end
        end
    endgenerate
endmodule

//============================================================
// local Xilinx two-read/one-write memory
//============================================================

// 本文件私有辅助模块：xilinx_2r1w是regfile使用的双读单写分布式RAM封装。
//-----------------------------------------------------------------
// Xilinx 2-read / 1-write distributed RAM
//-----------------------------------------------------------------


// [阶段3新增] 一个逻辑写口、两个完全独立的异步读口。
// RAM16X1D每个原语只保存16个地址的一位，因此32位数据需要横向放32个，
// x0-x15与x16-x31再各用一组。两个读口各自拥有副本，写入时同时更新，
// 从而避免用32个寄存器后接两棵很深的大MUX。
module xilinx_2r1w(
    input  wire        clk,
    input  wire        we,
    input  wire [4:0]  waddr,
    input  wire [31:0] wdata,
    input  wire [4:0]  raddr0,
    input  wire [4:0]  raddr1,
    output wire [31:0] rdata0,
    output wire [31:0] rdata1
);
    // 这个小RAM封装的名字看着像器件原语，其实它就是“一个写口、两个读口”的32位寄存器存储块。
    // spo_unused*接的是RAM16X1D的同步读出口SPO，本设计不用它，只用异步读出口DPO，所以统一收起来免lint报警。
    // r0_lo/r0_hi是读口0在低16个寄存器/高16个寄存器里的读数，r1_lo/r1_hi同理给读口1。
    // we_lo/we_hi按waddr[4]把x1~x15和x16~x31分开写；x0仍被挡在写使能外面。
    wire [31:0] spo_unused0;
    wire [31:0] spo_unused1;
    wire [31:0] spo_unused2;
    wire [31:0] spo_unused3;
    wire [31:0] r0_lo;
    wire [31:0] r0_hi;
    wire [31:0] r1_lo;
    wire [31:0] r1_hi;
    wire we_lo = we && (waddr != 5'b0) && !waddr[4];
    wire we_hi = we && (waddr != 5'b0) &&  waddr[4];

    genvar bit_no;
    generate
        for (bit_no = 0; bit_no < 32; bit_no = bit_no + 1) begin : GEN_RAM_BIT
            RAM16X1D u_r0_lo (
                .WCLK(clk), .WE(we_lo),
                .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]),
                .D(wdata[bit_no]),
                .DPRA0(raddr0[0]), .DPRA1(raddr0[1]),
                .DPRA2(raddr0[2]), .DPRA3(raddr0[3]),
                .DPO(r0_lo[bit_no]), .SPO(spo_unused0[bit_no])
            );
            RAM16X1D u_r1_lo (
                .WCLK(clk), .WE(we_lo),
                .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]),
                .D(wdata[bit_no]),
                .DPRA0(raddr1[0]), .DPRA1(raddr1[1]),
                .DPRA2(raddr1[2]), .DPRA3(raddr1[3]),
                .DPO(r1_lo[bit_no]), .SPO(spo_unused1[bit_no])
            );
            RAM16X1D u_r0_hi (
                .WCLK(clk), .WE(we_hi),
                .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]),
                .D(wdata[bit_no]),
                .DPRA0(raddr0[0]), .DPRA1(raddr0[1]),
                .DPRA2(raddr0[2]), .DPRA3(raddr0[3]),
                .DPO(r0_hi[bit_no]), .SPO(spo_unused2[bit_no])
            );
            RAM16X1D u_r1_hi (
                .WCLK(clk), .WE(we_hi),
                .A0(waddr[0]), .A1(waddr[1]), .A2(waddr[2]), .A3(waddr[3]),
                .D(wdata[bit_no]),
                .DPRA0(raddr1[0]), .DPRA1(raddr1[1]),
                .DPRA2(raddr1[2]), .DPRA3(raddr1[3]),
                .DPO(r1_hi[bit_no]), .SPO(spo_unused3[bit_no])
            );
        end
    endgenerate

    assign rdata0 = raddr0[4] ? r0_hi : r0_lo;
    assign rdata1 = raddr1[4] ? r1_hi : r1_lo;
endmodule

// 本文件私有辅助模块：RAM16X1D是iverilog/Verilator仿真用的最小原语模型。
// Vivado综合时由器件原语替代，regfile的读写接口和行为保持一致。
// [阶段3新增] 开源仿真器没有Xilinx UNISIM库，提供与原语时序一致的最小模型。
// Vivado综合时不会定义IVERILOG或VERILATOR，因而仍直接使用器件RAM16X1D原语。
`ifdef IVERILOG
// Icarus Verilog使用这一份行为模型：异步读、同步写，与Xilinx原语接口一致。
module RAM16X1D(DPO, SPO, A0, A1, A2, A3, D,
                DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE);
    parameter INIT = 16'h0000;
    output wire DPO, SPO;
    input wire A0, A1, A2, A3, D, DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE;
    // mem保存16个1bit小格子；wa是写/同步读地址，DPRA*拼出的地址用于第二个异步读口DPO。
    reg [15:0] mem;
    wire [3:0] wa = {A3, A2, A1, A0};
    assign SPO = mem[wa];
    assign DPO = mem[{DPRA3, DPRA2, DPRA1, DPRA0}];
    initial mem = INIT;
    always @(posedge WCLK)
        if (WE) mem[wa] <= D;
endmodule
`elsif VERILATOR
// 对于Verilator仿真，同样没有UNISIM库，因此复用等价行为模型，保证CI和NEMU联调可直接编译。
module RAM16X1D(DPO, SPO, A0, A1, A2, A3, D,
                DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE);
    parameter INIT = 16'h0000;
    output wire DPO, SPO;
    input wire A0, A1, A2, A3, D, DPRA0, DPRA1, DPRA2, DPRA3, WCLK, WE;
    // mem保存16个1bit小格子；wa是写/同步读地址，DPRA*拼出的地址用于第二个异步读口DPO。
    reg [15:0] mem;
    wire [3:0] wa = {A3, A2, A1, A0};
    assign SPO = mem[wa];
    assign DPO = mem[{DPRA3, DPRA2, DPRA1, DPRA0}];
    initial mem = INIT;
    always @(posedge WCLK)
        if (WE) mem[wa] <= D;
endmodule
`endif
