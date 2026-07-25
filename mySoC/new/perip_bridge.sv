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

module perip_bridge (
    input  logic         clk				, // CPU主时钟
    input  logic         cnt_clk			, // 固定50MHz计时参考时钟
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

    logic [31:0] LED;
    logic [31:0] seg_wdata, cnt_rdata, mmio_rdata, dram_rdata;
    logic [39:0] seg_output;

    // 新增：用于防亚稳态的两级同步寄存器（运行在CPU主时钟clk）
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

    // 2. 写入路由器：CPU写数据时的数据流向
    // 对照文档要求：外设区域只能4字节对齐访问，所以注释写了 we don't care perip_mask，因为对这里的硬件读写总是整个 32 位一起操作的。
    // we don't care perip_mask in LED, SEG, SW & KEY, only care in DRAM
    // write process
    // 每当时钟上升沿，如果 CPU 说“我要写数据”（perip_we 不为 0）：
    always_ff @(posedge clk) begin
        if (perip_en && (perip_we != 4'b0000)) begin
            case (perip_addr) // 检查 CPU 给出的地址
                // 如果地址命中了 0x8020_0040，说明 CPU 想操作 LED！
                // 那就把 CPU 传过来的数据 (perip_wdata) 直接锁死进 LED 寄存器里。
                LED_ADDR:   LED <= perip_wdata; 
                
                // 如果命中 0x8020_0020，说明给数码管写数字
                SEG_ADDR:   seg_wdata <= perip_wdata;
            endcase
            // 你会发现，这里没有写 SW 和 KEY！
            // 为什么？因为你回头看图片文档，SW 和 KEY 写了巨大的两个字：“只读”！
            // 开关是你手拨的，CPU当然不能去修改开关的物理状态。
        end
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
        .cpu_clk            (clk),
        .ref_clk            (cnt_clk),
        .rst                (rst),
        .perip_wdata		(perip_wdata),
        // 同理，只有地址完美等于 0x8020_0050 且有写使能，定时器才会理会 CPU 发来的打火/熄火命令。
        .cnt_wen 			(perip_en & (perip_we != 4'b0000) & (perip_addr == CNT_ADDR)),
        .perip_rdata		(cnt_rdata)
    );


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
        {32{perip_addr_read == CNT_ADDR}} & cnt_rdata;      // 0

    // 最后一大堆 0 和唯一一个有效的数据做 OR (|)，完美实现了类似 `switch case` 的多路选择器接线！
    
    // 把内部缓冲好的 LED 和 SEG 回填到给顶层的端口上。
    assign virtual_led_output = LED;
    assign virtual_seg_output = seg_output;

endmodule
