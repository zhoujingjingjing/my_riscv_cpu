`timescale 1ns / 1ps
`include "mycpu_top.h"

// 分支预测器 (BTB + RAS) 模块
// 原理：利用历史记录预测立刻要执行的指令是否发生跳转，避免流水线因为等待真实跳转结果而产生停顿。
module btb #(
    // 6位索引，意味着表格有 2^6 = 64 行。
    parameter INDEX_LEN = 6, 
    // 15位标签。我们不比较完整的32位PC，只截取中间的15位用于核对身份，这能大幅度减小比较器的电路延迟，提升CPU主频。
    parameter TAG_LEN   = 15 
)(
    input  wire        clk,
    input  wire        reset,

 
    /*第一部分：给 IF 级使用的预测查询端口 (要求极速，纯组合逻辑打通)*/
    input  wire [31:0] if_pc,        // 当前准备取指的PC地址

    // 预测器的输出结论
    output wire        pre_taken,    // 结论1：预测是否跳转？(1为跳，0为不跳)
    output wire [31:0] pre_target,   // 结论2：预测的跳转目标地址是多少？
    output wire        pre_is_ret,   // 结论3：这条指令是不是一个函数返回指令？
    output wire [INDEX_LEN-1:0] pre_index, // 结论4：当前预测是看了表里第几行得出的？(传给后级，算错账时方便找回来修改)
    /*==========================================================*/


    /*第二部分：给 ID 级使用的 RAS 栈维护端口 (时序逻辑)*/
    // RAS (返回地址栈) 专门用来处理函数返回时的地址。因为函数去哪里被调用是不固定的，返回地址也不能写死，必须用后进先出的栈来存。
    input  wire        id_push_ras,  // 当 ID 级翻译出这是一条“函数调用”指令，拉高此信号进栈
    input  wire [31:0] id_ras_wdata, // 要压入栈的返回地址 (通常是调用指令的 PC + 4)
    input  wire        id_pop_ras,   // 当 ID 级翻译出这是一条“函数返回”指令，拉高此信号出栈废弃一条地址
    /*==========================================================*/


    /*第三部分：给 EXE 级使用的 BTB 修正与更新端口 (时序逻辑)*/
    // 当指令走到 EXE 级，真实的跳转情况也就水落石出了。EXE 级把真实数据送回来，让预测器吃一堑长一智
    input  wire        exe_we,       // 更新使能。当 EXE 确认当前确实是一条分支指令时，拉高此信号开始写表
    input  wire [INDEX_LEN-1:0] exe_index, // 当初 IF 级查的是哪一行？现在就覆写哪一行
    input  wire [TAG_LEN-1:0]   exe_tag,   // 这条指令的真实身份标签 (tag匹配错误的时候修正)
    input  wire        exe_is_ret,   // 这条指令到底是不是 jalr 函数返回指令？
    input  wire        exe_taken,    // EXE 级算出的真实方向 (1为跳，0为不跳)
    input  wire [31:0] exe_target,    // EXE 级算出的真实目标地址 (预测没跳实际跳了或预测跳了实际没跳时需要修正)
    /*==========================================================*/



    // ========== 【新增：后悔药机制端口】 ==========
    output wire [2:0] current_ras_ptr,      // 吐出当前真实的指针给 IF 阶段
    input  wire       flush_en,         // 接收 EXE 阶段的清空警告 (出错啦！)
    input  wire [2:0] exe_restore_ras_ptr   // 接收 EXE 阶段扔回来的后悔药
);

   
    /*内部物理存储部件的定义*/
    // 直接映射 BTB 表 (64行)
    localparam BTB_SIZE = 1 << INDEX_LEN; // 1 左移 6 位，即 64
    
    reg               btb_valid   [BTB_SIZE-1:0]; // 有效位：记录这行是不是空数据。0代表空，1代表里面存过有效预测
    reg [TAG_LEN-1:0] btb_tag     [BTB_SIZE-1:0]; // 标签表：存下对应指令的高位片段，防止不同地址的指令恰好抢到了同一行
    reg [31:0]        btb_target  [BTB_SIZE-1:0]; // 目标表：存下普通分支指令上一次的跳转目的地址
    reg [1:0]         btb_counter [BTB_SIZE-1:0]; // 状态表：两位饱和计数器。用于统计历史跳转规律(本来属于PHT，但为了节省资源把它和BTB合在一起)
    reg               btb_is_ret  [BTB_SIZE-1:0]; // 标记表：标记这条指令是不是 jalr (专门的函数返回指令ret)

    // RAS 返回地址栈 (8个深度的硬件栈)
    reg [31:0] ras_stack [7:0]; // 8行的内部数组，用来存 PC+4。
    reg [2:0]  ras_ptr;         // 一个 3二进制位 的环形指针(范围0~7)，永远指向栈内下一个可以写入的空白行。
    /*==========================================================*/

    // 【修改1：直接用导线把内部指针接出去，让外部随时可见】
    assign current_ras_ptr = ras_ptr;


    /*IF 阶段预测逻辑 (纯零延迟组合逻辑)*/
    
    // 指令地址(PC)最低2位永远为 00(因为指令始终以4字节对齐)。因此最低2位是废位，不用看。
    // 我们从第 2 位开始往上取 6 位，即 [7:2]，作为去表格里查数据的行号(Index)。
    wire [INDEX_LEN-1:0] if_idx = if_pc[INDEX_LEN+1 : 2]; 
    
    // 再往上取 15 位，即 [22:8]，作为这条指令的身份证明(Tag)。
    wire [TAG_LEN-1:0]   if_tag = if_pc[TAG_LEN+INDEX_LEN+1 : INDEX_LEN+2]; 
    
    // 命中(match)的条件必须同时满足：这行不是空的 (valid为1) 且 这行记录的Tag和当前正在取的PC的Tag一模一样。
    wire match = btb_valid[if_idx] && (btb_tag[if_idx] == if_tag);
    
    // 如果命中了，且表里记录这恰好是一条用于“函数返回”的指令，则置起 is_ret_hit。
    wire is_ret_hit = match && btb_is_ret[if_idx]; 

    // 输出跳不跳结论：必须命中，且（是返回指令，100%跳；或者是普通跳转指令，且2位状态机的最高位是1，这表示倾向于跳）。
    assign pre_taken = match && (btb_is_ret[if_idx] || btb_counter[if_idx][1]);
    
    // 取出最新的有效返回地址：因为 ras_ptr 永远指向"下一个空位"，所以最新存进去的有效数据实际上在 ras_ptr - 1 的位置。
    // '& 3'b111' 是为了防止 0 减 1 变成负数产生越界，让它在 0~7 之间自动循环。
    wire [31:0] current_ras_top = ras_stack[(ras_ptr - 1'b1) & 3'b111];
    
    // 输出目标地址结论：如果是函数返回产生的命中，直接把栈里掏出来的返回地址输出；否则，老老实实输出表格里存的目标地址。
    assign pre_target = is_ret_hit ? current_ras_top : btb_target[if_idx];
    
    // 把当拍算出来的索引和性质同步送给流水线，记账用。
    assign pre_is_ret = is_ret_hit;
    assign pre_index  = if_idx;
    /*==========================================================*/



    /*ID 阶段：RAS 进栈与出栈 (时序逻辑)*/
    always @(posedge clk) begin
        if (reset) begin
            ras_ptr <= 3'b0; // 复位时，指针指向第0行。

        end else if (flush_en) begin// --- 新增：最高优先级！一旦EXE发现走错了，强行覆盖指针 
            ras_ptr <= exe_restore_ras_ptr; // 管家吃下后悔药，指针瞬间恢复

        end else if (id_push_ras) begin
                // 把紧跟着函数调用指令的下一条指令(PC+4)塞进正在指向的空行。
                ras_stack[ras_ptr] <= id_ras_wdata;
                // 指针上移一格，指向下一个空位。
                ras_ptr <= ras_ptr + 1'b1;
        end else if (id_pop_ras) begin
                // 函数运行完毕开始返回。指针下退一格，代表刚刚那个返回地址已经被用掉并且作废了。
                ras_ptr <= ras_ptr - 1'b1;
            
        end
    end
    /*==========================================================*/


    /*EXE 阶段：BTB 历史记录查漏补缺与更新 (时序逻辑)*/
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            // 复位时排空全表。有效位清零，饱和计数器设为强烈不跳转(2'b00)。
            for (i=0; i<BTB_SIZE; i=i+1) begin
                btb_valid[i]   <= 1'b0;
                //计数器默认悬停在弱不跳 (01),，给它一个缓冲
                btb_counter[i] <= 2'b01;//这里不需要去清零 target 或 tag，因为只要 valid 是 0，那些数据根本不会被读取，节省资源
            end
        end else if (exe_we) begin
            // 当该条指令在 EXE 确认自身为分支指令，就开始修改表里当初用来给自己预测的那一行的数据。
            btb_valid[exe_index]  <= 1'b1;
            btb_tag[exe_index]    <= exe_tag;
            btb_target[exe_index] <= exe_target;
            btb_is_ret[exe_index] <= exe_is_ret;
            
            // =======================================================
            // 2位饱和计数器的设计原理：
            // 共有 00(强不跳), 01(弱不跳), 10(弱跳), 11(强跳) 四个状态。
            // 只要真实发生一次跳，状态就加1；反之减1。上下限分别顶死在00和11，不会发生反转。
            // =======================================================

            // 我们判断一下："现在修改的这行指令，原来就是我自己吗？"
            if (btb_tag[exe_index] == exe_tag) begin
                if (exe_taken) begin // 如果真实行为是【跳了】
                    if (btb_counter[exe_index] != 2'b11) 
                        // 只要没到最强状态，就加一阶鼓励它。
                        btb_counter[exe_index] <= btb_counter[exe_index] + 2'b01;

                end else begin // 如果真实行为是【没跳】
                    if (btb_counter[exe_index] != 2'b00) 
                        // 只要没到最弱状态，就减一阶打压它。
                        btb_counter[exe_index] <= btb_counter[exe_index] - 2'b01;
                end
            end else begin
                // 【新来的鸠占鹊巢】：这是第一次入库，旧数据全是不相干的
                // 入库时它如果跳了，默认直接给 10(弱跳)；没跳，默认给 01(弱不跳)。
                btb_counter[exe_index] <= exe_taken ? 2'b10 : 2'b01;
            end
        end
    end
endmodule
