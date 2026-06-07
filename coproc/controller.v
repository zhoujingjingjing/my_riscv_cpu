// ============================================================================
// controller.v  （路线乙·改法A 版本）
// ----------------------------------------------------------------------------
// 这个模块是整台加速器的“大脑/总调度”。它本身不算数，只负责按顺序发命令：
// 先把权重和数据从 shared_buffer 搬进 weight/input buffer，再让 PE 阵列算，
// 最后把结果搬出去。
//
// 【本次改动的核心】
// 原版状态机有个致命问题：EN 一拉高，它就立刻自动开始读 shared_buffer，
// 根本不给 CPU 留“慢慢往 shared_buffer 里灌数据”的时间。
// （原版是在仿真里用 $readmemb 把数据预先塞进内存的，所以不需要等。）
//
// 现在我们要让 CPU 来灌数据，所以新增一个 LOADING（装填暂停）状态：
//   IDLE → LOADING（停在这里，把 shared_buffer 的写权限交给外面的 CPU）
//        → 等 CPU 灌完、给一个 load_done 信号 → 才继续 INPUTSW...一路自动跑完
//
// 为此新增了几个对外接口（下面有逐个解释），让 CPU 在 LOADING 期间能直接
// 控制 shared_buffer 的“写地址 / 写使能”。
// ============================================================================
 
module controller(
    input  wire        CLK,
    input  wire        RESET,
    input  wire        EN,
    output reg  [5:0]  STATE,        // 当前状态，送给外面看（CPU 靠读它判断算完没）
    output reg         W_EN,
    output reg         SELECTOR,
    input  wire [12:0] IADDR,        // 输入数据在 shared_buffer 里的起始地址
    input  wire [12:0] WADDR,        // 权重在 shared_buffer 里的起始地址
    input  wire [12:0] OADDR,        // 输出地址
 
    // ---- 下面这些是控制 shared_buffer 的信号 ----
    output reg         share_wen,
    output reg         share_ren,
    output reg         share_cen,
    output reg  [12:0] share_addr,
 
    output reg         weight_wen,
    output reg         weight_ren,
    output reg         weight_cen,
    output reg  [12:0] weight_addr,
    output reg         activate_wen,
    output reg         activate_ren,
    output reg         activate_cen,
    output reg  [12:0] activate_addr,
    output reg         output_wen,
    output reg         output_ren,
    output reg         output_cen,
    output reg  [12:0] output_addr,
 
    // ========================================================================
    // 【新增接口】给 CPU 在 LOADING 阶段灌数据用
    // ========================================================================
    // load_start：CPU 想开始装填时，给一个高电平脉冲。
    //   状态机看到它，就从 IDLE 进入 LOADING（停下来等 CPU 喂）。
    input  wire        load_start,
 
    // load_we / load_addr：CPU 在 LOADING 期间，
    //   要把数据写到 shared_buffer 的哪个地址(load_addr)、要不要写(load_we)。
    //   说明：进了 LOADING 后，shared_buffer 的写地址和写使能不再由状态机控制，
    //   而是直接“借”给 CPU，由这两根线说了算。
    input  wire        load_we,      // 1 = CPU 正在写 shared_buffer
    input  wire [12:0] load_addr,    // CPU 要写的 shared_buffer 地址
 
    // load_done：CPU 把整批数据都灌完了，给一个高电平脉冲。
    //   状态机看到它，才离开 LOADING、开始真正干活。
    input  wire        load_done,
 
    // calc_done：算完一整轮、回到 IDLE 时，给外面一个“完成”标志。
    //   CPU 可以读它（或读 STATE==IDLE）来判断结果可以取了。
    output reg         calc_done
    );
 
// ---- 原有状态编号 ----
parameter IDLE          = 6'd0;
parameter INPUTA        = 6'd1;
parameter INPUTW        = 6'd2;
parameter INPUTSW       = 6'd3;
parameter INPUTSA       = 6'd4;
parameter CALCULATE     = 6'd5;
parameter OUTPUT        = 6'd6;
parameter RETURN        = 6'd7;
parameter OUTPUTTOSHARE = 6'd8;
 
// 【新增状态】装填暂停区。编号接在后面，用 9。
parameter LOADING       = 6'd9;
 
always @(posedge CLK or negedge RESET) begin
    if(~RESET) begin
        // ---- 复位：全部回到初始，和原版一致 ----
        STATE        <= IDLE;
        W_EN         <= 0;
        SELECTOR     <= 0;
        share_wen    <= 1;
        share_ren    <= 0;
        share_cen    <= 1;
        share_addr   <= 0;
        weight_wen   <= 1;
        weight_ren   <= 0;
        weight_cen   <= 1;
        weight_addr  <= 0;
        activate_wen <= 1;
        activate_ren <= 0;
        activate_cen <= 1;
        activate_addr<= 0;
        output_wen   <= 1;
        output_ren   <= 0;
        output_cen   <= 1;
        output_addr  <= 0;
        calc_done    <= 0;   // 新增信号也要复位
    end else if (EN) begin
 
        // ====================================================================
        // 状态 IDLE：原版是“EN 一高直接冲去读数据”。
        // 现在改成：在 IDLE 里等 CPU 的 load_start。
        //   - 收到 load_start → 进 LOADING（停下来等喂数据）
        //   - 没收到就一直待在 IDLE
        // ====================================================================
        if (STATE == IDLE) begin
            calc_done <= 0;          // 新一轮开始，先把完成标志清掉
            if (load_start) begin
                STATE <= LOADING;
            end
        end
 
        // ====================================================================
        // 【新增状态】LOADING：装填暂停区，全篇改动的核心。
        // 在这里，状态机自己“按兵不动”，把 shared_buffer 的写控制交给 CPU：
        //   - share_cen <= 0  : 选中 shared_buffer（让它工作）
        //   - share_wen <= load_we : 写不写，听 CPU 的 load_we
        //   - share_ren <= 1  : 这个内存模块里 RETN(=ren) 要为高，读写才生效
        //   - share_addr<= load_addr : 写到哪个地址，听 CPU 的 load_addr
        // CPU 把数据一笔笔写进来；写完后拉高 load_done，状态机才放行。
        // ====================================================================
        else if (STATE == LOADING) begin
            // [修复] 装填期间 shared_buffer 的写控制已在 accelerator.v 顶层直接接 CPU(load_*)，
            //   controller 不再驱动 share_*（避免地址被寄存器化而与数据错位、以及写极性反掉）。
            //   这里只负责等 CPU 的 load_done。
            if (load_done) begin
                // [修复] CPU 已把 weight(@WADDR) 和 activate(@IADDR) 直接写进 shared_buffer，
                //   原版的 INPUTSW/INPUTSA 是"边读 input_data 边写 shared"的装载态，CPU 方案下
                //   不再需要、且会用陈旧数据覆盖刚装好的内容 → 直接跳到 INPUTW(读 shared→weight buffer)。
                //   入口条件对齐原版 INPUTSA→INPUTW：share 切到读模式、share_addr=WADDR、weight_addr=-1。
                STATE      <= INPUTW;
                share_wen  <= 1;         // 读模式（buffer: ~CEN & RETN 才读）
                share_ren  <= 1;
                share_cen  <= 0;
                share_addr <= WADDR;
                weight_addr<= -1;
            end
        end
 
        // ====================================================================
        // 从这里往下，到 RETURN 之前，全部是【原版逻辑，未改动】。
        // 它们负责：把权重/数据从 shared 搬到 weight/input buffer → 算 → 出结果。
        // ====================================================================
        else if(STATE == INPUTSW)begin
            share_addr <= share_addr + 1;
            if(share_addr >= 16 + WADDR)begin
                STATE      <= INPUTSA;
                share_addr <= IADDR;
            end
        end
        else if(STATE == INPUTSA)begin
            share_addr <= share_addr + 1;
            if(share_addr == 15 + IADDR)begin
                STATE      <= INPUTW;
                share_wen  <= 1;
                share_ren  <= 1;
                share_cen  <= 0;
                share_addr <= WADDR;
                weight_addr<= -1;
            end
        end
        else if (STATE == INPUTW)begin
            weight_wen <= 0;
            weight_ren <= 1;
            weight_cen <= 1;
            share_addr <= share_addr + 1;
            weight_addr<= weight_addr + 1;
            if(share_addr == 16 + WADDR)begin
                STATE        <= INPUTA;
                share_addr   <= IADDR;
                weight_wen   <= 1;
                weight_ren   <= 1;
                weight_cen   <= 0;
                weight_addr  <= -1;
                activate_addr<= -1;
                SELECTOR      = 1;
                W_EN          = 1;
            end
        end
        else if (STATE == INPUTA)begin
            activate_wen <= 0;
            activate_ren <= 1;
            activate_cen <= 1;
            share_addr   <= share_addr + 1;
            activate_addr<= activate_addr + 1;
            weight_addr  <= weight_addr + 1;
            if(share_addr == 16 + IADDR)begin
                STATE        <= CALCULATE;
                share_wen    <= 1;
                share_ren    <= 0;
                share_cen    <= 1;
                activate_wen <= 1;
                activate_ren <= 1;
                activate_cen <= 0;
                activate_addr<= -1;
            end
        end
        else if (STATE == CALCULATE)begin
            W_EN     = 0;
            SELECTOR = 0;
            activate_addr <= activate_addr + 1;
            // [修复] 激活按对角斜移喂入，16x16 需流满 31 条对角线(mem[0..30])才能让
            //   全部 16 个输出行拿到完整点积；原来只流到 16 → 只有前 ~2 行数据完整。
            if(activate_addr == 30)begin
                STATE        <= OUTPUT;
                activate_wen <= 1;
                activate_ren <= 0;
                activate_cen <= 1;
                output_addr  <= 0;
                output_wen   <= 0;
                output_ren   <= 1;
                output_cen   <= 1;
            end
        end
        else if (STATE == OUTPUT)begin
            output_addr <= output_addr + 1;
            if(output_addr == 30)begin
                STATE <= RETURN;
            end
        end
 
        // ====================================================================
        // 状态 RETURN：原版直接回 IDLE。
        // 这里【加一行】：回 IDLE 的同时，把 calc_done 拉高，
        // 告诉 CPU“算完了，可以来取结果了”。
        // ====================================================================
        else if (STATE == RETURN)begin
            STATE     <= IDLE;
            calc_done <= 1'b1;   // 新增：完成标志
        end
    end
end
endmodule
 