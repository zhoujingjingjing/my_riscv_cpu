`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 05:18:59 PM
// Design Name: 
// Module Name: uart
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

module uart #(
    parameter CLK_FREQ = 50000000,
    parameter BAUD_RATE = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] rx_data,
    output reg rx_ready,

    output reg tx,
    input wire [7:0] tx_data,
    input wire tx_start,
    output reg tx_busy
);
    // 计算波特率分频系数。例如 50M / 115200 ≈ 434。
    // 意味着每过 434 个系统时钟周期，串口线上才发送/接收 1 个比特(bit)的数据。
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;

    reg [1:0] rx_state;
    reg [12:0] rx_cnt;
    reg [7:0] rx_shift;
    // 定义3个寄存器用来“打拍”，消除亚稳态
    reg rx_d0, rx_d1, rx_d2;

    always @(posedge clk or negedge rst_n) begin
        // 这里是为了防止外部输入的单根 rx 信号带有毛刺，或者和系统时钟不同步。
        // 用3个触发器连续寄存3次。最后用稳定的 rx_d2 作为内部处理信号。
        if (!rst_n) begin
            rx_d0 <= 1'b1;
            rx_d1 <= 1'b1;
            rx_d2 <= 1'b1;
        end else begin
            rx_d0 <= rx;     // 第一拍
            rx_d1 <= rx_d0;  // 第二拍
            rx_d2 <= rx_d1;  // 第三拍：经过稳定处理后的接收电平
        end
    end

    // 寻找下降沿：上一拍 rx_d2 是1，当前拍 rx_d1 突然变成0。
    // 原理：串口空闲时是高电平(1)，一旦拉低(0)就代表来了“起始位”（发令枪响）。
    wire rx_negedge = rx_d2 & ~rx_d1;

    reg [3:0] rx_bit_cnt;// 用来数已经接收了多少个数据位了（0~7）
    reg rx_ready_pulse;// 内部信号：当一帧数据接收完成时，拉高一个周期的脉冲，告诉外面“数据准备好了！”
    reg [15:0] rx_ready_cnt;//

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= 0;
            rx_cnt <= 0;
            rx_bit_cnt <= 0;
            rx_shift <= 0;
            rx_data <= 0;
            rx_ready_pulse <= 0;
        end else begin
            rx_ready_pulse <= 0;
            case(rx_state)
                0: begin // IDLE: 等待状态
                    if(rx_negedge) begin // 探测到下降沿（收到起始位了）
                        rx_state <= 1;   // 切换到下一个状态
                        // 把秒表（rx_cnt）提前装载为全周期的一半（BAUD_DIV >> 1 就是除以2）
                        // 相当于先跑了半拍，那等状态1再走满整个BAUD_DIV周期时，
                        // 时间刚好卡在下一个起始位的正中间！这就是对齐正中间的绝妙手法。
                        rx_cnt <= BAUD_DIV >> 1;
                        rx_bit_cnt <= 0; // 准备开始数第0个数据位
                    end
                end
                1: begin // 过渡阶段：从【起始位】开始走到【起始位】正中间
                    if(rx_cnt == BAUD_DIV-1) begin // 等这剩下来的那“半个波特率周期”走完
                        rx_cnt <= 0;     // 此时正好对应起始位的正中央，秒表清零
                        rx_state <= 2;   
                    end else
                        rx_cnt <= rx_cnt + 1; // 还没到一半，秒表还在走
                end
                2: begin // 开始循环 8 次读取数据位 (对应图中的状态大长条)
                    if(rx_cnt == BAUD_DIV-1) begin // 走完一整个波特率周期
                                                   // 此时正好对应图上的一个【位采样】尖刺！抓取在数据的正中间！
                        rx_cnt <= 0; 
                        
                        // 开始偷偷拿数据！rx_d2 就是当前稳定后的线上电平。
                        // 移位方式：新采到的位放在最高位，老数据集体右移。
                        rx_shift <= {rx_d2, rx_shift[7:1]};
                        
                        if(rx_bit_cnt == 7) // 已经连抓了 8 次（位0~位7全部拿完）
                            rx_state <= 3;  // 数据收全了，准备打包交货
                        else
                            rx_bit_cnt <= rx_bit_cnt + 1; // 还没拿够，【位计数】加 1
                    end else
                        rx_cnt <= rx_cnt + 1; 
                end
                3: begin // 结束状态，处理【停止位】与输出确认脉冲
                    if(rx_cnt == BAUD_DIV-1) begin // 再等最后一个周期（也就是等停止位走完）
                        rx_cnt <= 0;
                        rx_state <= 0;           // 任务圆满结束，回到空闲状态，等下一帧
                        
                        rx_data <= rx_shift;     // 输出数据：把拼好的 8bit 放出去交差
                        rx_ready_pulse <= 1'b1;  // 输出使能：拉高一个尖峰脉冲，告诉外面有新数据！
                    end else begin
                        rx_cnt <= rx_cnt + 1;
                    end
                end
                default: rx_state <= 0;
            endcase
        end
    end
    


    /*把 rx_ready_pulse 展宽成 rx_ready：
    我们在 状态3 结束时，产生了一个 rx_ready_pulse = 1;。
    这个 pulse，即所谓的系统时钟单周期脉冲。在 50MHz 的时钟下，它只存在 20纳秒 就立刻变回 0 了*/

    // rx_ready delay for half of BAUD_DIV
    // 将一闪而过的单周期脉冲 (rx_ready_pulse)，展宽为一个持续较长时间的高电平 (rx_ready)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_ready <= 1'b0;      // 展宽后的有效信号
            rx_ready_cnt <= 0;     // 用来计时的沙漏
        end else begin
            if (rx_ready_pulse) begin // 只要那个短暂的脉冲一出现！
                rx_ready <= 1'b1;     // 立刻把对外的标识线拉高
                rx_ready_cnt <= 0;    // 开始沙漏倒计时
            end else if (rx_ready) begin // 如果现在正处于拉高的状态中
                // 只要还没数满 (BAUD_DIV - 1)，就一直保持拉高！
                // 这相当于强行让这个有效信号维持了一个完整的波特率周期（代码注释写的是 half，但实际代码逻辑是维持了一整个 BAUD_DIV）
                if (rx_ready_cnt < BAUD_DIV - 1) begin
                    rx_ready_cnt <= rx_ready_cnt + 1;
                end else begin
                    rx_ready <= 1'b0; // 时间到，拉低收工
                end
            end
        end
    end


    /*UART发送协议
    我们要发送的 tx_data 只有 8 个位。
    电脑要求：在你发这 8 个位之前，必须先给我发一个 0（拉低电平：起始位）。
    发完这 8 个位之后，必须发一个 1（拉高电平：停止位），整个过程总共需要 10 个位的时间*/
    // tx state machine
    reg [3:0] tx_state;      // 状态机变量：0=空闲，1=正在发送
    reg [12:0] tx_cnt;       // 掐表：用来等够一个波特率周期时间
    reg [3:0] tx_bit_cnt;    // 记件数：数数看 10次 发完了没有？
    reg [9:0] tx_shift;      // 这里不是 8位了！而是 10 位的发件箱缓冲区

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= 0;
            tx_busy <= 0;
            tx_cnt <= 0;
            tx_bit_cnt <= 0;
            tx_shift <= 10'b1111111111;//平时空闲时发件箱全装满 1 
            tx <= 1'b1;//极其重要！串口空闲时线上的默认电平必须是高电平
        end else begin
            case(tx_state)
                0: begin // TX IDLE: 空闲状态
                    tx_busy <= 0;
                    if(tx_start) begin // 外部告诉我要发送数据了
                        // 拼装 10 个比特：低位 0 (起始发令枪)，中间 tx_data (真实数据)，高位 1 (停止位)
                        tx_shift <= {1'b1, tx_data, 1'b0};
                        tx_state <= 1; // 切换到发送状态
                        tx_cnt <= 0;
                        tx_bit_cnt <= 0;
                        tx_busy <= 1;  // 拉高 busy，告诉外面：“我正在忙，别发新东西来”
                    end
                end
                1: begin // SEND: 发送状态 (把 10 个位依次推到发路线上)
                    if(tx_cnt == BAUD_DIV-1) begin // 同样是那个经典的计时器判断，当等待时间刚好度过了一个波特周期的物理时间(433)
                        tx_cnt <= 0;
                        tx <= tx_shift[0]; // 把最右侧的最低位（比如一开始的那个0），推到物理接口 TX 上，让外界的电平跟着变
                        tx_shift <= {1'b1, tx_shift[9:1]}; //队列最高位塞一个空的高电平1进去，所有位全部往右边移一步
                        if(tx_bit_cnt == 9) // 0-9  10 个位发完了
                            tx_state <= 0;  // 完事了，休息
                        else
                            tx_bit_cnt <= tx_bit_cnt + 1; // 还没发完就接着发
                    end else
                        tx_cnt <= tx_cnt + 1;
                end
            endcase
        end
    end

endmodule
