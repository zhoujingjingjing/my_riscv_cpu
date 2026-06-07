`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/04/22 10:25:24
// Design Name: 
// Module Name: perip_bridge
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
// ============================================================================
// perip_bridge.sv  （路线乙·改法A 版本）
// ----------------------------------------------------------------------------
// 这是 CPU 的“外设插座墙”。CPU 想操作任何外设，都是往某个地址(门牌号)写数/读数，
// 由这个桥根据地址把请求转给对应外设。原本已经挂了 SW/KEY/SEG/LED/DRAM/计数器。
//
// 【本次改动】在这面墙上新增一个“协处理器(矩阵乘加速器)”插座。难点是：
//   - CPU 一次只能送 32 位，但加速器的 shared_buffer 一次要 128 位。
//     → 解决：在桥里放一个 128 位“攒数寄存器”，CPU 连写 4 个 32 位把它填满，
//       满了就自动写进加速器的 shared_buffer，并把写地址 +1。
//   - 加速器要按 start→灌数→done→算→读结果 的流程走。
//     → 解决：给它分配几个门牌号，CPU 写不同门牌号就触发不同动作。
//
// 协处理器门牌号分配（落在外设区 0x8020_0060 起的空白处，4字节对齐）：
//   0x8020_0060  写：压入一个32位数据（连写4次=128位，自动入 shared_buffer，地址+1）
//   0x8020_0064  写1：开始装填 (load_start)
//   0x8020_0068  写1：装填完毕、启动计算 (load_done)
//   0x8020_006C  读：bit0=calc_done(算完没), bit[6:1]=STATE
//   0x8020_0070  写：把结果读指针归零并预读
//   0x8020_0074  读：取结果（连读4次=把128位结果取走）
// ============================================================================
module perip_bridge(
    input  logic         clk				, // CPU主时钟
    input  logic         cnt_clk			, // 计数器外设时钟
    input  logic         rst                , // 复位信号

    input  logic         perip_en           , // CPU总使能
    input  logic [31:0]  perip_addr			, // CPU给出的外设访问地址
    input  logic [31:0]  perip_wdata		, // CPU想写入的数据
    input  logic [3:0]	 perip_we			, // CPU字节写掩码（替代原来的wen和mask）
    output logic [31:0]  perip_rdata		, // 桥返回给CPU的读取数据

    input  logic [63:0]  virtual_sw_input	, // 从Twin Controller传来的虚拟拨码开关输入
    input  logic [7:0]   virtual_key_input	, // 从Twin Controller传来的虚拟按键输入	

	output logic [39:0]  virtual_seg_output	, // 送给Twin Controller的虚拟数码管输出
    output logic [31:0]  virtual_led_output   // 送给Twin Controller的虚拟LED输出
);
    // 1. 查字典：地址本的定义（对照图片中的地址映射表）
    // 对应左图黄色的 DRAM 区域：大小为 256KB (文档里的 0x8010_0000 ~ 0x8013_FFFF)
    localparam DRAM_ADDR_START = 32'h8010_0000;
    localparam DRAM_ADDR_END   = 32'h8013_FFFF;

    // 以下是右图的 MIMO(MMIO)外设区域：0x8020_0000 起始
    localparam SW0_ADDR  = 32'h8020_0000;  // 对应文档：SW低32位，读取 sw[31:0]
    localparam SW1_ADDR  = 32'h8020_0004;  // 对应文档：SW高32位，读取 sw[63:32]
    localparam KEY_ADDR  = 32'h8020_0010;  // 对应文档：KEY区域，只读8位
    localparam SEG_ADDR  = 32'h8020_0020;  // 对应文档：SEG(数码管)区域，读写
    localparam LED_ADDR  = 32'h8020_0040;  // 对应文档：LED区域，只写
    localparam CNT_ADDR  = 32'h8020_0050;  // 对应文档：计数器区域，读写

    // ========================================================================
    // 【新增】协处理器门牌号
    // ========================================================================
    localparam COP_PUSH_ADDR  = 32'h8020_0060; // 写：压入32位数据
    localparam COP_START_ADDR = 32'h8020_0064; // 写：开始装填
    localparam COP_DONE_ADDR  = 32'h8020_0068; // 写：装填完毕、启动计算
    localparam COP_STAT_ADDR  = 32'h8020_006C; // 读：状态
    localparam COP_RRST_ADDR  = 32'h8020_0070; // 写：结果读指针归零
    localparam COP_RDAT_ADDR  = 32'h8020_0074; // 读：取结果

    logic [31:0] LED;
    logic [31:0] seg_wdata, cnt_rdata, mmio_rdata, dram_rdata;
    logic [39:0] seg_output;

    // ---- 原有：拨码/按键两级同步，防亚稳态 ----
    // 新增：用于防亚稳态的两级同步寄存器 (运行在 180MHz clk)
    logic [63:0] virtual_sw_sync1, virtual_sw_sync2;
    logic [7:0]  virtual_key_sync1, virtual_key_sync2;

    // 新增：2-Stage DFF 消除亚稳态
    always_ff @(posedge clk) begin
        if (rst) begin
            virtual_sw_sync1  <= 64'd0;
            virtual_sw_sync2  <= 64'd0;
            virtual_key_sync1 <= 8'd0;
            virtual_key_sync2 <= 8'd0;
        end else begin
            // 第一拍：可能产生亚稳态
            virtual_sw_sync1  <= virtual_sw_input;
            virtual_key_sync1 <= virtual_key_input;
            // 第二拍：亚稳态基本消除，这组信号可以安全给下面逻辑使用
            virtual_sw_sync2  <= virtual_sw_sync1;
            virtual_key_sync2 <= virtual_key_sync1;
        end
    end


    // ========================================================================
    // 【新增】协处理器相关的内部寄存器和线
    // ========================================================================
    // 攒数寄存器：把 CPU 连写的 4 个 32 位拼成 128 位
    logic [127:0] cop_load_buf;     // 攒数缓冲
    logic [1:0]   cop_word_cnt;     // 已攒了几个字（0~3）
    logic [12:0]  cop_load_addr;    // 当前要写入 shared_buffer 的地址
    logic         cop_load_we;      // 这一拍是否真正写 shared_buffer
    logic [127:0] cop_load_wdata;   // 真正送给加速器的 128 位数据
    logic         cop_load_start;   // 开始装填脉冲
    logic         cop_load_done;    // 装填完毕脉冲
 
    // 加速器送出来的信号
    logic [5:0]   cop_state;        // 加速器当前状态
    logic         cop_calc_done;    // 算完标志
    logic [127:0] cop_result;       // 128 位结果
 
    // 取结果用：把 128 位结果按 32 位一段读出去
    logic [1:0]   cop_read_cnt;     // 读到第几段（0~3）
 
    // ========================================================================
    // 【新增】写逻辑：CPU 往协处理器门牌号写东西时的处理
    // ========================================================================

    // 2. 写入路由器：CPU写数据时的数据流向
    // 对照文档要求：外设区域只能4字节对齐访问，所以注释写了 we don't care perip_mask，因为对这里的硬件读写总是整个 32 位一起操作的。
    // we don't care perip_mask in LED, SEG, SW & KEY, only care in DRAM
    // write process
    // 每当时钟上升沿，如果 CPU 说“我要写数据”（perip_we 不为 0）：
    always_ff @(posedge clk) begin
        if (rst) begin
            cop_load_buf   <= 128'd0;
            cop_word_cnt   <= 2'd0;
            cop_load_addr  <= 13'd0;
            cop_load_we    <= 1'b0;
            cop_load_wdata <= 128'd0;
            cop_load_start <= 1'b0;
            cop_load_done  <= 1'b0;
        end else begin
            // 这两个是“脉冲”信号，默认每拍清零，只有命中时才拉高一拍
            cop_load_start <= 1'b0;
            cop_load_done  <= 1'b0;
            cop_load_we    <= 1'b0;   // 写 shared_buffer 也是单拍动作

            if (perip_en && (perip_we != 4'b0000)) begin
                case (perip_addr) // 检查 CPU 给出的地址
                    // 如果地址命中了 0x8020_0040，说明 CPU 想操作 LED！
                    // 那就把 CPU 传过来的数据 (perip_wdata) 直接锁死进 LED 寄存器里。
                    LED_ADDR:   LED <= perip_wdata; 
                    
                    // 如果命中 0x8020_0020，说明给数码管写数字
                    SEG_ADDR:   seg_wdata <= perip_wdata;

                    // -------- 协处理器：压入一个 32 位数据 --------
                    // 把新来的 32 位拼到攒数寄存器的对应段。
                    // 攒满 4 个（cop_word_cnt 到 3）时，这一拍就把 128 位写进
                    // shared_buffer（拉高 cop_load_we），并把写地址 +1、计数清零。
                    COP_PUSH_ADDR: begin
                        // 按 word_cnt 决定塞到 128 位的哪一段（低段先塞）
                        case (cop_word_cnt)
                            2'd0: cop_load_buf[31:0]    <= perip_wdata;
                            2'd1: cop_load_buf[63:32]   <= perip_wdata;
                            2'd2: cop_load_buf[95:64]   <= perip_wdata;
                            2'd3: cop_load_buf[127:96]  <= perip_wdata;
                        endcase
 
                        if (cop_word_cnt == 2'd3) begin
                            // 第 4 个字到位：拼出完整 128 位，触发写入
                            cop_load_wdata <= {perip_wdata, cop_load_buf[95:0]};
                            cop_load_we    <= 1'b1;            // 写 shared_buffer
                            cop_load_addr  <= cop_load_addr + 1'b1; // 地址递增
                            cop_word_cnt   <= 2'd0;            // 重新开始攒下一组
                        end else begin
                            cop_word_cnt <= cop_word_cnt + 1'b1;
                        end
                    end
 
                    // -------- 协处理器：开始装填 --------
                    // 装填前把地址和计数复位，并给加速器一个 load_start 脉冲。
                    COP_START_ADDR: begin
                        cop_load_start <= 1'b1;
                        cop_load_addr  <= 13'd0;
                        cop_word_cnt   <= 2'd0;
                    end
 
                    // -------- 协处理器：装填完毕、启动计算 --------
                    COP_DONE_ADDR: begin
                        cop_load_done <= 1'b1;
                    end
 
                    // -------- 协处理器：结果读指针归零 --------
                    COP_RRST_ADDR: begin
                        cop_read_cnt <= 2'd0;
                    end
                endcase
                // 你会发现，这里没有写 SW 和 KEY！
                // 为什么？因为你回头看图片文档，SW 和 KEY 写了巨大的两个字：“只读”！
                // 开关是你手拨的，CPU当然不能去修改开关的物理状态。
            end
        end    
    end


    // ========================================================================
    // 【新增】读结果：每读一次结果门牌号，就把指针 +1，下次读下一段
    // ========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            cop_read_cnt <= 2'd0;
        end else if (perip_en && (perip_we == 4'b0000) &&
                     (perip_addr == COP_RDAT_ADDR)) begin
            cop_read_cnt <= cop_read_cnt + 1'b1;
        end
    end
 
    // 把 128 位结果按当前指针切出 32 位
    logic [31:0] cop_result_word;
    always_comb begin
        case (cop_read_cnt)
            2'd0: cop_result_word = cop_result[31:0];
            2'd1: cop_result_word = cop_result[63:32];
            2'd2: cop_result_word = cop_result[95:64];
            2'd3: cop_result_word = cop_result[127:96];
        endcase
    end


    // 3. 读取路由器：CPU读数据时的数据汇聚 (修改：增加打拍寄存器对齐 BRAM 延迟)
    // 如果 CPU 不是要写，而是要拿数据，那就根据地址把对应的数据挂载到返回总线上。
    // read process: in one cycle
    logic [31:0] perip_addr_read;
    always_ff @(posedge clk) begin
        perip_addr_read <= perip_addr; // 把地址打一拍，留给下个周期提取数据
    end

    // 原来的读逻辑，现在使用延迟一拍的地址 (perip_addr_read) 进行选择
    always_comb begin
            case (perip_addr_read)
                // 巧妙的数据切分：文档说 SW 有 64 位，但 32 位 CPU 一次只能读 32 位。
                // 所以访问 0x00 给你低 32 位，访问 0x04 (+4字节的位置) 给你高 32 位。
                SW0_ADDR:  mmio_rdata = virtual_sw_sync2[31:0];
                SW1_ADDR:  mmio_rdata = virtual_sw_sync2[63:32];
                
                // KEY只有8位，剩下24位补0
                KEY_ADDR:  mmio_rdata = {24'd0, virtual_key_sync2};
                
                // 读数码管当前的显示数值 (读写属性)
                SEG_ADDR:  mmio_rdata = seg_wdata; 
                
                // 如果 CPU 瞎访问一个没有定义的地址（比如 0x8020_0030）
                // 就返回一个经典黑客梗 0xDEADBEEF ("死牛肉")，方便看波形时迅速知道读错了。
                default:   mmio_rdata = 32'hDEAD_BEEF; 
            endcase
    end

    // 4. 衍生外设子模块的挂载 (挂载数码管、DRAM 和 定时器)
    // 总线桥除了分配上面几个简单的寄存器，还要挂载那些行为比较复杂的“黑盒子”。

    // seg driver
    // 数码管扫描驱动器：把 CPU 刚写进 seg_wdata 的几十位数据，变成真实的动态扫描脉冲
    display_seg seg_driver (
        .clk    (clk),
        .rst    (rst),
        .s      (seg_wdata),
        .seg1   (seg_output[6:0]),
        .seg2   (seg_output[16:10]),
        .seg3   (seg_output[26:20]),
        .seg4   (seg_output[36:30]),
        .ans    ({seg_output[39:38], seg_output[29:28], seg_output[19:18], seg_output[9:8]})
    ); 
   
    assign seg_output[7]  = 0; // 这些是数码管的小数点位，直接写死了不亮
    assign seg_output[17] = 0;
    assign seg_output[27] = 0;
    assign seg_output[37] = 0;
    

    // dram rw (直接挂载内存 BRAM IP)
    // 只有 CPU 访问地址 >= 0x8010_0000 且 <= 0x8013_FFFF (完美对应图2的DRAM写保护区间) 才会选通
    wire dram_en = perip_en & (perip_addr >= DRAM_ADDR_START && perip_addr <= DRAM_ADDR_END);
    
    data_ram Mem_DRAM (
        .clka        (clk),
        .ena         (dram_en),
        .wea         (perip_we),               // 直接塞入4位字节掩码
        .addra       (perip_addr[17:2]),       // BRAM只认Word地址，扔掉低2位
        .dina        (perip_wdata),
        .douta       (dram_rdata)              // 数据自动滞后一拍输出
    );


    // counter rw (挂载计时器)
    counter counter_inst (
        .clk				(clk),
        .rst                (rst),
        .perip_wdata		(perip_wdata),
        // 同理，只有地址完美等于 0x8020_0050 且有写使能，定时器才会理会 CPU 发来的打火/熄火命令。
        .cnt_wen 			(perip_en & (perip_we != 4'b0000) & (perip_addr == CNT_ADDR)),
        .perip_rdata		(cnt_rdata)
    );



    // ========================================================================
    // 【新增】实例化加速器（已改造的 Accelerator）
    // ========================================================================
    // 说明：
    //   - EN 直接给 1（常开），真正的“开始/装填/算”由 load_start/load_done 驱动。
    //   - RESET 用低有效：这套加速器内部是 negedge RESET 复位，所以接 ~rst。
    //   - IADDR/WADDR/OADDR：先用固定值。权重放 shared_buffer 地址 0 起，
    //     数据紧跟其后。这里给 WADDR=0、IADDR=16、OADDR=0（与 controller 里
    //     “16 个一批”的节奏对应）。如需调整按数据布局改。
    Accelerator u_accel (
        .CLK        (clk),
        .RESET      (~rst),
        .EN         (1'b1),
        .IADDR      (13'd16),
        .WADDR      (13'd0),
        .OADDR      (13'd0),
        .STATE      (cop_state),
        .load_start (cop_load_start),
        .load_we    (cop_load_we),
        .load_addr  (cop_load_addr),
        .load_wdata (cop_load_wdata),
        .load_done  (cop_load_done),
        .calc_done  (cop_calc_done),
        .result_data(cop_result)
    );
 
    // 协处理器状态读出值：bit0=算完没，bit[6:1]=状态机当前状态
    logic [31:0] cop_stat_word;
    assign cop_stat_word = {25'd0, cop_state, cop_calc_done};

    // 5. 最终数据总线大合并 (多路选择器，全篇最精华的硬件语法)
    // 最后，我们要把前面算好的 mmio_rdata (开关按键数码管读取值)、dram_rdata (内存读取值)、cnt_rdata (定时器读取值)，这三股数据选出一个，还给 CPU。
    // 代码使用了一种极其硬核且底层的 “独热码 OR-MUX (大跨度或逻辑复用器)” 写法：
    assign perip_rdata = 
        // 解释： {32{条件}} 是 Verilog 语法糖。如果条件成立，就是 32 个 1 (即 0xFFFF_FFFF)，
        // 跟后面的数据做 按位与(&) 时，数据原样保留。 
        // 如果条件不成立，就是 32 个 0，跟后面的数据与，结果全为 0。

        // 举例：假如当前 perip_addr_read 刚好是 0x8020_0004 (SW1_ADDR)。
        {32{perip_addr_read == SW0_ADDR}} & mmio_rdata |    // 0
        {32{perip_addr_read == SW1_ADDR}} & mmio_rdata |    // -> 这个条件成立，变 0xFFFF_FFFF，保留数据
        {32{perip_addr_read == KEY_ADDR}} & mmio_rdata |    // 0
        {32{perip_addr_read == SEG_ADDR}} & mmio_rdata |    // 0
        // ... 判断是不是在 DRAM 范围内 ...
        {32{perip_addr_read >= DRAM_ADDR_START && perip_addr_read <= DRAM_ADDR_END}} & dram_rdata | // 0
        // ... 判断是不是定时器 ...
        {32{perip_addr_read == CNT_ADDR}} & cnt_rdata|     // 0
        // 【新增】协处理器的两个读地址
        {32{perip_addr_read == COP_STAT_ADDR}} & cop_stat_word |
        {32{perip_addr_read == COP_RDAT_ADDR}} & cop_result_word;
    // 最后一大堆 0 和唯一一个有效的数据做 OR (|)，完美实现了类似 `switch case` 的多路选择器接线！
    
    // 把内部缓冲好的 LED 和 SEG 回填到给顶层的端口上。
    assign virtual_led_output = LED;
    assign virtual_seg_output = seg_output;

endmodule
