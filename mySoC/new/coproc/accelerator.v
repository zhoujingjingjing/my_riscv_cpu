// ============================================================================
// accelerator.v  （路线乙·改法A 版本）
// ----------------------------------------------------------------------------
// 这是加速器的“总装车间”：把 controller（大脑）、shared_buffer（总仓库）、
// weight/input/output buffer（小仓库）、PE_array（16x16 脉动阵列，计算心脏）
// 全部接线组装成一台完整机器。
//
// 【本次改动】
// 1) 新增一批对外接口（load_*、calc_done、load_wdata），让外面的 CPU/总线
//    能够：在装填阶段直接把 128 位数据写进 shared_buffer，并控制装填的开始/结束。
// 2) 把这些信号接到内部的 controller 和 shared_buffer 上。
//
// 关键点：shared_buffer 的“数据输入口 D”现在接的是 load_wdata（CPU 灌进来的
// 128 位数据），而它的“地址/写使能”在 LOADING 阶段由 controller 转交给 CPU 的
// load_addr/load_we 控制（这套转交逻辑写在 controller.v 里）。
// ============================================================================
 
module Accelerator(
    input  wire        CLK,
    input  wire        RESET,
    input  wire        EN,
 
    input  wire [12:0] IADDR,             // 输入数据在 shared_buffer 的起始地址
    input  wire [12:0] WADDR,             // 权重在 shared_buffer 的起始地址
    input  wire [12:0] OADDR,             // 输出起始地址
    output wire [5:0]  STATE,             // 当前状态，送出去给 CPU 看
 
    // ========================================================================
    // 【新增接口】装填相关（供总线/CPU 使用）
    // ========================================================================
    input  wire        load_start,        // CPU：开始装填（脉冲）
    input  wire        load_we,           // CPU：装填期间是否在写
    input  wire [12:0] load_addr,         // CPU：装填写到哪个 shared_buffer 地址
    input  wire [127:0] load_wdata,       // CPU：要写进 shared_buffer 的 128 位数据
    input  wire        load_done,         // CPU：装填完毕（脉冲）
    output wire        calc_done,         // 加速器：算完了（给 CPU 看）
 
    // 取结果用：把 output_buffer 当前读出的数据引出去（128 位）
    output wire [127:0] result_data,      // 算完后，结果从这里读（保留旧口，Phase2 再统一）

    // [新增] 干净的去斜结果读口：CPU 给行号 result_raddr，读回该行 16 个 16bit(共256位)
    input  wire [3:0]   result_raddr,
    output wire [255:0] result_row,

    // [新增·bring-up调试口] 把 PE 阵列底行的 256 位原始累加和(16个16bit)引出，
    //   仅用于独立仿真验证 MAC 是否正确；接入 CPU 时可忽略此口。
    output wire [255:0] dbg_out_sum,
    // [新增·bring-up调试口] PE 阵列的实际输入：激活向量(input_buffer.Q)、权重向量(weight_buffer.Q)、
    //   以及 share_out / 控制信号，用来判断 bug 在"喂数据"还是"阵列本身"。
    output wire [127:0] dbg_input_out,
    output wire [127:0] dbg_weight_out,
    output wire [127:0] dbg_share_out,
    output wire         dbg_w_en,
    output wire         dbg_selector
    );
 
// ---------------- controller ----------------
wire [12:0] share_addr;
wire        W_EN;
wire        SELECTOR;
wire        share_wen;
wire        share_ren;
wire        share_cen;
wire        weight_ren;
wire        weight_cen;
wire        weight_wen;
wire [12:0] weight_addr;
wire        input_ren;
wire        input_cen;
wire        input_wen;
wire [12:0] input_addr;
wire        output_ren;
wire        output_cen;
wire        output_wen;
wire [12:0] output_addr;
 
controller controller(
        .CLK(CLK),
        .RESET(RESET),
        .EN(EN),
        .STATE(STATE),
        .W_EN(W_EN),
        .SELECTOR(SELECTOR),
        .share_wen(share_wen),
        .share_ren(share_ren),
        .share_cen(share_cen),
        .share_addr(share_addr),
        .weight_wen(weight_wen),
        .weight_ren(weight_ren),
        .weight_cen(weight_cen),
        .weight_addr(weight_addr),
        .activate_wen(input_wen),
        .activate_ren(input_ren),
        .activate_cen(input_cen),
        .activate_addr(input_addr),
        .output_wen(output_wen),
        .output_ren(output_ren),
        .output_cen(output_cen),
        .output_addr(output_addr),
        .IADDR(IADDR),
        .WADDR(WADDR),
        .OADDR(OADDR),
 
        // 【新增】把装填接口接进 controller
        .load_start(load_start),
        .load_we(load_we),
        .load_addr(load_addr),
        .load_done(load_done),
        .calc_done(calc_done)
    );
 
// ---------------- shared buffer（总仓库）----------------
// 【关键改动】它的数据输入口 D 现在接 load_wdata：
//   也就是 CPU 在装填阶段灌进来的 128 位数据。
//   （原版这里接的是顶层一个叫 input_data 的口，现在统一改成 load_wdata。）
wire [127:0] share_out;
// [修复] 装填路径重构：LOADING 期间 shared_buffer 的写控制直接由 CPU(load_*) 驱动，
//   绕开 controller 的寄存器化 share_addr（原来会让地址比数据晚一拍 → 错位），
//   并修正写极性（buffer 是 ~WEN 才写，CPU 的 load_we=1 表示要写 → WEN=~load_we）。
//   非 LOADING 期间仍用 controller 的 share_* 信号（读 shared 喂 weight/input buffer）。
wire        loading_sb = (STATE == 6'd9);   // 9 = LOADING
wire        sb_cen  = loading_sb ? 1'b1      : share_cen;
wire        sb_wen  = loading_sb ? ~load_we  : share_wen;
wire [12:0] sb_addr = loading_sb ? load_addr : share_addr;
wire        sb_retn = loading_sb ? 1'b1      : share_ren;
shared_buffer share_buffer(
    .Q(share_out),
    .CLK(CLK),
    .CEN(sb_cen),
    .WEN(sb_wen),
    .A(sb_addr),
    .D(load_wdata),          // CPU 灌进来的数据从这里进总仓库
    .RETN(sb_retn)
);
 
// ---------------- input buffer（数据小仓库）----------------
wire [127:0] input_out;
input_buffer input_buffer(
    .Q(input_out),
    .CLK(CLK),
    .CEN(input_cen),
    .WEN(input_wen),
    .A(input_addr),
    .D(share_out),
    .RETN(input_ren),
    .RESET(RESET)
);
 
// ---------------- weight buffer（权重小仓库）----------------
wire [127:0] weight_out;
weight_buffer weight_buffer(
    .Q(weight_out),
    .CLK(CLK),
    .CEN(weight_cen),
    .WEN(weight_wen),
    .A(weight_addr),
    .D(share_out),
    .RETN(weight_ren)
);
 
// ---------------- PE array（16x16 脉动阵列，计算心脏）----------------
parameter num1 = 16;
parameter num2 = 16;
wire [num2*16-1:0] out_sum;
wire [num2*8-1:0]  out_weight_below;
PE_array #(.num1(num1),.num2(num2)) PE_array(
        .CLK(CLK),
        .RESET(RESET),
        .EN(EN),
        .SELECTOR(SELECTOR),
        .W_EN(W_EN),
        .active_left(input_out),
        .out_sum_final(out_sum),
        .in_weight_above(weight_out),
        .out_weight_final(out_weight_below)
    );
assign dbg_out_sum   = out_sum;   // [新增·bring-up] 引出原始累加和供独立仿真核对
assign dbg_input_out = input_out;
assign dbg_weight_out= weight_out;
assign dbg_share_out = share_out;
assign dbg_w_en      = W_EN;
assign dbg_selector  = SELECTOR;
 
// ---------------- output buffer（成品仓库）----------------
// 【改动】把它读出的数据 Q 引到顶层 result_data，供 CPU 取结果。
wire [127:0] output_out;
output_buffer output_buffer(
    .Q(output_out),
    .CLK(CLK),
    .CEN(output_cen),
    .WEN(output_wen),
    .A(output_addr),
    .D(out_sum),
    .RETN(output_ren)
);
 
assign result_data = output_out;   // 结果引出去（旧口）

// ============================================================================
// [新增] 干净的去斜结果捕获 + 按行读出
// ----------------------------------------------------------------------------
// 结果在 out_sum 上以对角线浮现：C[m][j]=out_sum[j] 出现在 (CALCULATE起拍+CAP_START+m+j)。
// 这里用一个计数器 calc_cnt 记录进入 CALCULATE 后的拍数，在 [CAP_START, CAP_START+2N-1)
// 窗口内拉高 cap_en，由 result_buffer 做对角散射去斜，存成可按行读的 16x16 结果。
// CAP_START 由独立仿真标定（阵列固有时延，固定值）。
// ============================================================================
localparam CAP_START = 6'd18;   // CALCULATE 起拍到结果浮现的时延（独立仿真标定）
reg [6:0] calc_cnt;
always @(posedge CLK or negedge RESET) begin
    if (~RESET)                                  calc_cnt <= 7'd0;
    else if (STATE == 6'd5 || STATE == 6'd6)     calc_cnt <= calc_cnt + 7'd1;  // 贯穿 CALCULATE(5)+OUTPUT(6)
    else                                         calc_cnt <= 7'd0;
end
// 捕获窗口需覆盖全部 2N-1=31 条对角线（结果从 CALCULATE 末尾一直浮现到 OUTPUT 中段）
wire cap_en = (STATE == 6'd5 || STATE == 6'd6) &&
              (calc_cnt >= CAP_START) && (calc_cnt < CAP_START + 7'd31);

result_buffer #(.N(16)) u_result_buffer (
    .CLK(CLK),
    .RESET(RESET),
    .cap_en(cap_en),
    .out_sum(out_sum),
    .result_raddr(result_raddr),
    .result_row(result_row)
);

endmodule