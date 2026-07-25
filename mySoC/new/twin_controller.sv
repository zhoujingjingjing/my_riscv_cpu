`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 06:04:59 PM
// Design Name: 
// Module Name: twin_controller
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


module twin_controller(
    input wire clk,
    input wire rst_n,

    input wire rx_ready,
    input wire [7:0] rx_data,

    output reg tx_start,
    output reg [7:0] tx_data,
    input wire tx_busy,

    output reg [63:0] sw,
    output reg [7:0] key,
    input wire [39:0] seg,
    input wire [31:0] led
);

    // 状态机定义：管家每天就干两件事，要么闲着等指令（IDLE），要么在发狂填信封（SEND）
    typedef enum reg [0:0] {
        IDLE = 1'd0,
        SEND = 1'd1
    } state_t;

    reg [4:0] send_cnt; // 记件数：数数看给电脑发了几封信了？（0~17 共 18 个信封）
    reg [7:0] status_buffer[0:17];// 专门用来装要发给电脑的数据,18 个信封，每个信封 8 位宽
    reg [7:0] tx_data_next;// 计划方案：下一步准备发什么数据给pc？先写在这个草稿纸上，等状态机切换到 SEND 的时候再正式发出
    reg tx_start_next;// 计划方案：告诉UART要开始接收数据了。会使tx_busy变1
    state_t current_state, next_state;

    // 三段式状态机的第一段：当前状态逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    // 三段式状态机的第二段：次态逻辑，决定下一个状态next_state 以及计算计划输出的信号（tx_start_next 和 要发送给UART的数据tx_data_next）
    // 这里的 tx_start_next 和 tx_data_next 是“计划方案”
    always @(*) begin
        next_state = current_state; // 1. 默认状态不变
        tx_start_next = 0;          // 2. 默认不发送（比如当前数码管值）给 UART
        tx_data_next = tx_data;     // 3. 默认发件箱内容不变

        case(current_state)
            IDLE: begin
                // rx_ready 是 UART 发来的输出脉冲：“收到一包新数据！”
                if(rx_ready) begin
                    // ---- 管家的协议密码本 ----
                    // 1. 如果信上写着 0x80 (1000_0000)：
                    if(rx_data == 8'h80) begin
                        next_state = SEND; // 这代表“拍照汇报指令”，切换到“发件模式”
                        tx_start_next = 0;//需要回信，但无需立即发件，等进入 SEND 状态后再发
                    // 2. 如果写着其他：
                    end else begin
                        next_state = IDLE; // 只是拨动开关指令，不用回信（发送状态数据给 UART），接着闲聊
                        if(rx_data[6:0] <= 72 && rx_data[6:0] >= 1) begin
                            tx_start_next = 0;// 不需要回信，直接控制开关就行了
                        end
                    end
                end
            end
            SEND: begin
                // 发件模式：只要 UART 不忙
                if(~tx_busy) begin
                    // 抽出第 send_cnt 号信封里的数据，写到计划草稿纸 tx_data_next准备给 UART
                    tx_data_next = status_buffer[send_cnt];  
                    // 把“要求UART开始接收数据”的计划写在 tx_start_next 上！
                    tx_start_next = 1;                      
                    // 如果 18个信封 (0~17) 全部送完，打卡下班回到 IDLE
                    if (send_cnt == 17)
                        next_state = IDLE;
                end
            end
            default: begin
                next_state = IDLE;
            end
        endcase
    end

    // 三段式状态机的第三段：输出逻辑
    //将算出来的计划输出数据 (_next)，盖章变成正式生效的寄存器输出！
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start <= 0;
            tx_data <= 8'd0;
        end else begin
            tx_start <= tx_start_next;
            tx_data <= tx_data_next;
        end
    end

    reg tx_start_d; // 打个拍子，存上一个时钟周期 tx_start 的状态

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start_d <= 0;
        end else begin
            tx_start_d <= tx_start; // 始终滞后 tx_start 一个时钟周期
        end
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            send_cnt <= 0;
        end else if(current_state == IDLE) begin
            send_cnt <= 0;
        end else if(current_state == SEND && tx_start && ~tx_start_d) begin
            send_cnt <= send_cnt + 1;//每当 tx_start 从0-1的上升沿，就把 send_cnt 加 1，准备发下一封信
        end
    end



    /*电脑数字孪生平台发数据控制Soc的拨码开关、按键寄存器
    (★其实就是你在孪生平台上拨开关sw，外设sw寄存器的值就变了| 如果接下来你myCPU读这个外设sw寄存器，就能知道开关状态了)

    rx_data[6:0] <= 72 && rx_data[6:0] >= 1 到底是什么暗号？
    这里其实是在讲解：电脑发出的一封信（8个比特 1 个字节），到底怎么翻译成对 64 个开关和 8 个按键的精准控制。

    我们知道，一个字节有 8 位，最高位是 [7]，剩下 7 位是 [6:0]

    在这个数字孪生系统里，设计者自己定了一个通信小协议：
        最高位 [7] 负责指示【开关状态】：它是 1 就代表开关往上拨/按键按下；是 0 就代表开关往下拨/按键松开。

        低 7 位 [6:0] 负责指示【器件编号】：这根信号到底是去控制谁？
            这 7个位 一共能表示多少个数字？ 2的7次方=128（从 0 到 127）。
            1 ~ 64 号：被指定分配给了 FPGA 上的 64 个大拨码开关。
            65 ~ 72 号：被指定分配给了 FPGA 上的 8 个小按键。
            （0 号不用；128 即 1000_0000 也就是 0x80，作为切换为发送状态SEND的指令）*/

    always @(posedge clk or negedge rst_n) begin
        // 场景 A：如果编号是 1 到 64
        if(!rst_n) begin
            sw <= 64'd0;
            key <= 8'd0;
        end
        else if(rx_data[6:0] <= 64)
            // 比如收到编号是 5 (rx_data[6:0] = 5)
            // 也就是想控制第 4 号开关（因为数组是 sw[0] 到 sw[63] 起步，所以要 5 - 1）
            // 于是把这8位数据里的最高位rx_data[7]赋值给 sw[4]
            sw[rx_data[6:0] - 1] <= rx_data[7];
            
        // 场景 B：如果编号是 65 到 72
        else if(rx_data[6:0] <= 72)
            // 比如收到编号 68 (rx_data[6:0] = 68)。
            // 减去基准偏移值 65，刚好等于 3 
            // 于是它就把这包数据里的最高位，写入到 key[3] 里面！
            key[rx_data[6:0] - 65] <= rx_data[7];
    end





    /*SoC把自己的状态更新给电脑数字孪生平台：数码管显示、LED灯状态、开关状态、按键状态
    （★比如myCPU执行指令把LED外设寄存器的值给更新了，那么数字孪生平台的LED就会点亮）

    在此之前有一个重大前提：
    我们这块实验板，CPU 操控的外设非常多！

    有 40 位长度的数码管控制线 (seg[39:0])
    有 64 位宽的拨码开关线 (sw[63:0])
    有 32 位宽的 LED 灯控制线 (led[31:0])
    加上 8 个按键 (key[7:0])。
    加起来总共 144 条线的数据，不可能靠一次 8 位传输全发给电脑，所以只能大卸八块，一片一片发。
    这段代码的核心，就是把长数据切断，存进 18 个“小信封”（status_buffer[17:0]），
    每个信封刚好 8 位宽，刚好能让 UART 快递员发走。*/
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 开机，或者按下了板子上的复位键时：
            // 将 18 个小信封里全部清空填 0
            for(i = 0; i < 18; i = i + 1)
                status_buffer[i] <= 8'd0;
                
        end else if(rx_ready && rx_data == 8'h80 && current_state == IDLE) begin
        // 关键条件判断：
        // 1. rx_ready：快递员敲门了，收到一包新数据！
        // 2. rx_data == 8'h80：我拆开信一看，里面写着 0x80（上位机问“当前各个灯和开关啥状态？”的特殊口令）
        // 3. current_state == IDLE：当前的管家闲着没事干，可以接客

            // 把 seg 的第 0 到第 7 条线，倒进 0号信封
            status_buffer[0]  <= seg[7:0];  
            // 把 seg 的第 8 到第 15 条线，倒进 1号信封
            status_buffer[1]  <= seg[15:8];  
            // 接下来依法炮制，直到装满 5 个信封。
            status_buffer[2]  <= seg[23:16];
            status_buffer[3]  <= seg[31:24];
            status_buffer[4]  <= seg[39:32]; 

            // 8 个按键刚好塞满一个信封，给 5 号。
            status_buffer[5]  <= key;

            // 拨码开关 sw: 64bit (64 / 8 = 需用 8 个信封)
            status_buffer[6]  <= sw[7:0];
            status_buffer[7]  <= sw[15:8];
            status_buffer[8]  <= sw[23:16];
            status_buffer[9]  <= sw[31:24];
            status_buffer[10] <= sw[39:32];
            status_buffer[11] <= sw[47:40];
            status_buffer[12] <= sw[55:48];
            status_buffer[13] <= sw[63:56]; 

            // 指示灯控制 led: 32bit (32 / 8 = 恰好用光最后 4 个)
            status_buffer[14] <= led[7:0];
            status_buffer[15] <= led[15:8];
            status_buffer[16] <= led[23:16];
            status_buffer[17] <= led[31:24]; // 到这刚刚好，总共装满了从第 0 到 17 编号的 18个信封！
        end
    end

endmodule

/* ★
1、数字孪生平台的LED如何被点亮：myCPU执行IROM里指令的过程中把LED外设寄存器的值给更新了，那么数字孪生平台的LED就会点亮
2、myCPU如何读取开关状态：直接读外设sw寄存器就能知道开关状态了。
但是这个sw寄存器的值又是从哪里来的呢？myCPU只能读sw寄存器，不能写sw寄存器，sw寄存器的值是在电脑数字孪生平台操作开关来更新的。

                         (3. 指令流动)
      +--------------+ <================ +---------+
      |              |                   |         |
      |     IROM     |                   |  myCPU  | (唯一的“大脑”！)
      | (存放代码的ROM)|                   |         | 
      |              | =================>|         |
      +--------------+    (4. 吐出指令)   +---------+
                                           |     ^
                                (1. 读写请求)|     |(2. 返回数据)
                                           V     |
                               +-----------------------------+
                               |                             |
                               |    perip_bridge (总线桥)    | (负责地址分发)
                               |                             |
                               +-----------------------------+
                                 /            |            \
                   (读写DRAM)    /             |             \   (送出虚拟LED/SEG)
                              /              | (接收SW/KEY)  \
                             V               |               V
                 +---------------+           |        +-----------------+
                 |  DRAM (内存)  |           |        | Twin Controller | 
                 +---------------+           +------- | (管家+外设寄存器)|
                                                      +-----------------+
                                                              |
                                                              | (字节流)
                                                              V
                                                      +-----------------+
                                                      |      UART       |
                                                      +-----------------+
                                                              ^
                                                              | (USB串口线)
                                                              V   
                                                      [ PC 电脑数字孪生平台 ]
*/